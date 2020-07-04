//
// Created by ubuntu-jianan on 6/11/20.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H
#include "slam_rgbd/common_include.h"
#include "slam_rgbd/frame.h"

namespace slamrgbd {
    class MapPoint {
    private:
        unsigned long id_;
        static unsigned long factory_id_; //factory id
        bool good_;
        Vector3d pos_;
        Vector3d norm_;
        Mat descriptor_;
        list<Frame::Ptr> observed_frames_;   // key frames that this feature is observed

        int matched_times_;   //being an inliner in a pose estimation
        int visible_times_;   //being visible in current frame

    public:
        typedef shared_ptr<MapPoint> Ptr;
        MapPoint() : id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), good_(true), visible_times_(0), matched_times_(0) {}
        MapPoint(unsigned long id, const Vector3d & position, const Vector3d & norm, Frame::Ptr & frame, const Mat & descriptor = Mat())
        : id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor) {
            observed_frames_.push_back(frame);
        }

        inline cv::Point3f GetPositionCV() const {
            return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }
        inline unsigned long GetId() {return id_;}

        static MapPoint::Ptr CreateMapPoint();
        static MapPoint::Ptr CreateMapPoint(const Vector3d & pos_world, const Vector3d & norm_, const Mat descriptor, Frame::Ptr & frame);
    };
    class Frame;
}


#endif //MYSLAM_MAPPOINT_H
