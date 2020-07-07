//
// Definition of Map class members
//

#include "slam_rgbd/map.h"

namespace slamrgbd {
    void Map::InsertKeyFrame(Frame::Ptr & frame) {
        keyframes_[frame->GetId()] = frame;
        // cout << "The keyframe size is " << keyframes_.size() << endl;
    }

    void Map::InsertMapPoint(MapPoint::Ptr & map_point) {
        map_points_[map_point->GetId()] = map_point;
        // cout << "The map size is " << map_points_.size() << endl;
    }
}

