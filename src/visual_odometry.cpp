//
// Declare the VO class
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "slam_rgbd/config.h"
#include "slam_rgbd/visual_odometry.h"
#include "slam_rgbd/g2o_types.h"

namespace slamrgbd {
    VisualOdometry::VisualOdometry() : state_(INITIALIZING), ref_(NULL), curr_(NULL), map_(new Map), num_lost_(0), num_inliers_(0), matcher_flann_(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2)) {
        num_of_features_    = Config::get<int> ( "number_of_features" );
        scale_factor_       = Config::get<double> ( "scale_factor" );
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        map_point_erase_ratio_ = Config::get<double> ("map_point_erase_ratio");
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    bool VisualOdometry::AddFrame(Frame::Ptr frame) {
        switch(state_) {
            case INITIALIZING:
            {
                state_ = OK;
                ref_ = curr_ = frame;
                ExtractKeyPoints();
                ComputeDescriptors();
                AddKeyFrame();
                break;
            }
            case OK:
            {
                curr_ = frame;
                curr_->GetTransformation() = ref_->GetTransformation();
                ExtractKeyPoints();
                ComputeDescriptors();
                FeatureMatching();
                PoseEstimationPnP();
                if (CheckEstimatedPose() == true) {
                    curr_->GetTransformation() = transform_matrix_c_r_estimated_;
                    OptimizeMap();
                    num_lost_ = 0;
                    if (CheckKeyFrame() == true) {
                        AddKeyFrame(); // Add if it is a key-frame
                    }
                }
                else {
                    num_lost_++;
                    if (num_lost_ > max_num_lost_) {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            }
            case LOST:
            {
                cout << " VO has lost!" << endl;
                break;
            }
        }
        return true;
    }

    void VisualOdometry::ExtractKeyPoints() {
        boost::timer timer;
        orb_->detect(curr_->GetColor(), keypoints_curr_);
        cout << "Extracting keypoints costs time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::ComputeDescriptors() {
        boost::timer timer;
        orb_->compute(curr_->GetColor(), keypoints_curr_, descriptors_curr_);
        cout << "Computing descriptors costs time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::FeatureMatching() {
        boost::timer timer;
        vector<cv::DMatch> matches;
        // Find the map points in view
        Mat desp_map;
        vector<MapPoint::Ptr> candidates;
        for (auto & point : map_->AccessMapPoints()) {
            MapPoint::Ptr & p = point.second;
            if (curr_->IsInFrame(p->GetPosition())) {
                p->GetVisibleTimes()++;
                candidates.push_back(p);
                desp_map.push_back(p->GetDescriptor());
            }
        }

        cout << desp_map.size() << endl;
        cout << descriptors_curr_.size() << endl;
        matcher_flann_.match(desp_map, descriptors_curr_, matches);
        float min_dis = min_element(matches.begin(), matches.end(), [] (const cv::DMatch & m1, const cv::DMatch & m2) {
            return m1.distance < m2.distance;
        })->distance;
        match_3dpts_.clear();
        match_2dkp_index_.clear();
        for (cv::DMatch & m : matches) {
            if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
                match_3dpts_.push_back(candidates[m.queryIdx]);
                match_2dkp_index_.push_back(m.trainIdx); //index
            }
        }

        cout << "Good matches: " << match_3dpts_.size() << endl;
        cout << "Match cost time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::PoseEstimationPnP() {
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for (int index : match_2dkp_index_) {
            pts2d.push_back(keypoints_curr_[index].pt);
        }
        for (MapPoint::Ptr pt : match_3dpts_) {
            pts3d.push_back(pt->GetPositionCV());
        }

        Mat K = ( cv::Mat_<double>(3,3)<<
                ref_->GetCamera()->GetFx(), 0, ref_->GetCamera()->GetCx(),
                0, ref_->GetCamera()->GetFy(), ref_->GetCamera()->GetCy(),
                0,0,1
        );

        Mat rvec, tvec, inliers;
        cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        num_inliers_ = inliers.rows;
        cout << "PnP inliers: " << num_inliers_ << endl;
        transform_matrix_c_r_estimated_ = SE3(
                Sophus::SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
                );

        // BA
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
        Block::LinearSolverType * linear_solver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block * solver_ptr = new Block(linear_solver);
        g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(g2o::SE3Quat(transform_matrix_c_r_estimated_.rotation_matrix(), transform_matrix_c_r_estimated_.translation()));
        optimizer.addVertex(pose);

        for (int i = 0; i < num_inliers_; ++i) {
            int index = inliers.at<int>(i, 0);
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId(i);
            edge->setVertex(0, pose);
            edge->SetCamera(curr_->GetCamera());
            edge->SetPoint( pts3d[index].x, pts3d[index].y, pts3d[index].z );
            edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
            edge->setInformation( Eigen::Matrix2d::Identity() );
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        transform_matrix_c_r_estimated_ = SE3(
          pose->estimate().rotation(),
          pose->estimate().translation());
    }

    void VisualOdometry::AddKeyFrame() {
        if (map_->AccessKeyframes().empty()) {
            for (int i = 0; i < keypoints_curr_.size(); ++i) {
                double depth = curr_->FindDepth(keypoints_curr_[i]);
                if (depth < 0) {
                    continue;
                }
                Vector3d p_world = ref_->GetCamera()->PixelToWorld(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->GetTransformation());
                Vector3d  n = p_world - ref_->GetCameraCenter(); // direction of viewing
                n.normalize();
                MapPoint::Ptr map_point = MapPoint::CreateMapPoint(p_world, n, descriptors_curr_.row(i).clone(), curr_);
                map_->InsertMapPoint(map_point);
            }
        }

        cout<<"Adding a key-frame"<<endl;
        map_->InsertKeyFrame ( curr_ );
        ref_ = curr_;
    }
    bool VisualOdometry::CheckEstimatedPose() {
        if (num_inliers_ < min_inliers_) {
            cout << "Reject because amount of inlier is too small: " << num_inliers_ << endl;
            return false;
        }
        Sophus::Vector6d d = transform_matrix_c_r_estimated_.log();
        if (d.norm() > 5.0) {
            cout << "Reject because motion is too large: " << d.norm() << endl;
            return false;
        }
        return true;
    }
    bool VisualOdometry::CheckKeyFrame() {
        Sophus::Vector6d d = transform_matrix_c_r_estimated_.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans) {
            return true;
        }
        return false;
    }

    void VisualOdometry::AddMapPoints() {
        vector<bool> matched(keypoints_curr_.size(), false);
        for (int index : match_2dkp_index_) {
            matched[index] = true;
        }
        for (int i = 0; i < keypoints_curr_.size(); ++i) {
            if (!matched[i]) {
                continue;
            }
            double depth = ref_->FindDepth(keypoints_curr_[i]);
            if (depth < 0) {
                continue;
            }
            Vector3d p_world = ref_->GetCamera()->PixelToWorld(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->GetTransformation(), depth);
            Vector3d n = p_world - ref_->GetCameraCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::CreateMapPoint(
                    p_world, n, descriptors_curr_.row(i).clone(), curr_);
            map_->InsertMapPoint(map_point);


        }
    }

    void VisualOdometry::OptimizeMap() {
        for (auto itr = map_->AccessMapPoints().begin(); itr != map_->AccessMapPoints().end();) {
            if (curr_->IsInFrame(itr->second->GetPosition()) == false) {
                itr = map_->AccessMapPoints().erase(itr);
                continue;
            }
            // Erase if the feature gets rarely matched
            float match_ratio = float(itr->second->GetMatchedTimes()) / itr->second->GetVisibleTimes();
            if (match_ratio < map_point_erase_ratio_) {
                itr = map_->AccessMapPoints().erase(itr);
                continue;
            }
            double angle = GetViewAngle(curr_, itr->second);
            if (angle > M_PI / 6) {
                itr = map_->AccessMapPoints().erase(itr);
                continue;
            }
            if (itr->second->IsGood() == false) {

            }
            itr++;
        }

        if (match_2dkp_index_.size() < 100) {
            AddMapPoints();
        }

        if (map_->AccessMapPoints().size() > 1000) {
            map_point_erase_ratio_ += 0.05;
        }
        else {
            map_point_erase_ratio_ = 0.1;
        }
        cout << "The number of map points is " << map_->AccessMapPoints().size() << endl;
    }

    double VisualOdometry::GetViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {
        Vector3d n = point->GetPosition() - frame->GetCameraCenter();
        n.normalize();
        return acos(n.transpose() * point->GetNorm());
    }
}

