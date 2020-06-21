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

namespace slamrgbd {
    VisualOdometry::VisualOdometry() : state_(INITIALIZING), ref_(NULL), curr_(NULL), map_(new Map), num_lost_(0), num_inliers_(0) {
        num_of_features_    = Config::get<int> ( "number_of_features" );
        scale_factor_       = Config::get<double> ( "scale_factor" );
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    bool VisualOdometry::AddFrame(Frame::Ptr frame) {
        switch(state_) {
            case INITIALIZING:
            {
                state_ = OK;
                curr_ = ref_ = frame;
                map_->InsertKeyFrame(frame);
                ExtractKeyPoints();
                ComputeDescriptors();
                SetRef3DPoints();
                break;
            }
            case OK:
            {
                curr_ = frame;
                ExtractKeyPoints();
                ComputeDescriptors();
                FeatureMatching();
                PoseEstimationPnP();
                if (CheckEstimatedPose() == true) {
                    curr_->GetTransformation() = transform_matrix_c_r_estimated_ * ref_->GetTransformation();
                    ref_ = curr_;
                    SetRef3DPoints();
                    num_lost_ = 0;
                    if (CheckKeyFrame() == true) {
                        AddKeyFrame();
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
        orb_->detect(curr_->GetColor(), keypoints_curr_);
    }

    void VisualOdometry::ComputeDescriptors() {
        orb_->compute(curr_->GetColor(), keypoints_curr_, descriptors_curr_);
    }

    void VisualOdometry::FeatureMatching() {
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(descriptors_ref_, descriptors_curr_, matches);
        float min_dis = min_element(matches.begin(), matches.end(), [] (const cv::DMatch & m1, const cv::DMatch & m2) {
            return m1.distance < m2.distance;
        })->distance;

        feature_matches_.clear();
        for (cv::DMatch & m : matches) {
            if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
                feature_matches_.push_back(m);
            }
        }
        cout << "good matches: " << feature_matches_.size() << endl;
    }

    void VisualOdometry::PoseEstimationPnP() {
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for (cv::DMatch m : feature_matches_) {
            pts3d.push_back(pts_3d_ref_[m.queryIdx]);
            pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
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

    }
    void VisualOdometry::SetRef3DPoints() {
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
        for (size_t i = 0; i < keypoints_curr_.size(); ++i) {
            double d = ref_->FindDepth(keypoints_curr_[i]);
            if (d > 0) {
                Vector3d p_cam = ref_->GetCamera()->PixelToCamera(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
                pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }

    void VisualOdometry::AddKeyFrame() {
        cout<<"adding a key-frame"<<endl;
        map_->InsertKeyFrame ( curr_ );
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
}

