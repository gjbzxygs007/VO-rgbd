//
// Declaration of VO class
//

#ifndef SLAMRGBD_VISUAL_ODOMETRY_H
#define SLAMRGBD_VISUAL_ODOMETRY_H
#include "slam_rgbd/common_include.h"
#include "slam_rgbd/map.h"
#include <opencv2/features2d/features2d.hpp>

namespace slamrgbd {
    class VisualOdometry {
    public:
        enum VOState {
            INITIALIZING = -1,
            OK = 0,
            LOST
        };
    private:
        VOState state_;  //current VO states
        Map::Ptr map_;  // map with all frames and map points
        Frame::Ptr ref_;  // reference frame
        Frame::Ptr curr_;  //current frame
        cv::Ptr<cv::ORB> orb_;  // orb detector and computer
        vector<cv::Point3f> pts_3d_ref_;        // 3d points in reference frame
        vector<cv::KeyPoint> keypoints_curr_;    // keypoints in current frame
        Mat descriptors_curr_;  // descriptor in current frame
        Mat descriptors_ref_;   // descriptor in reference frame
        vector<cv::DMatch> feature_matches_; // matched features;
        SE3 transform_matrix_c_r_estimated_;  // the estimated pose of current frame
        int num_inliers_;        // number of inlier features in icp
        int num_lost_;           // number of lost times
        double map_point_erase_ratio_;

        // parameters
        int num_of_features_;   // number of features
        double scale_factor_;   // scale in image pyramid
        int level_pyramid_;     // number of pyramid levels
        float match_ratio_;      // ratio for selecting  good matches
        int max_num_lost_;      // max number of continuous lost times
        int min_inliers_;       // minimum inliers

        double key_frame_min_rot;   // minimal rotation of two key-frames
        double key_frame_min_trans; // minimal translation of two key-frames


    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        VisualOdometry();
        ~VisualOdometry() = default;
        inline VOState GetState() {return state_;}
        bool AddFrame(Frame::Ptr frame);

    protected:
        void ExtractKeyPoints();
        void ComputeDescriptors();
        void FeatureMatching();
        void PoseEstimationPnP();
        void SetRef3DPoints();

        void AddKeyFrame();
        bool CheckEstimatedPose();
        bool CheckKeyFrame();

    };

}

#endif //SLAMRGBD_VISUAL_ODOMETRY_H
