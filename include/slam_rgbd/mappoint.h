//
// Created by ubuntu-jianan on 6/11/20.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H
#include "slam_rgbd/common_include.h"

namespace slamrgbd {
    class MapPoint {
        private:
        unsigned long id_;
        Vector3d pos_;
        Vector3d norm_;
        Mat descriptor_;
        int observed_times_;
        int correct_times_;

        public:
        typedef shared_ptr<MapPoint> Ptr;
        MapPoint() : id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), observed_times_(0), corrent_times_(0) {};
        MapPoint(long id, Vector3d position, Vector3d norm);

        static MapPoint::Ptr CreateMapPoint();
    };
    class Frame;
}


#endif //MYSLAM_MAPPOINT_H
