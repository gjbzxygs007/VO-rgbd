//
// Definition of Map class members
//

#include "slam_rgbd/map.h"

namespace slamrgbd {
    void Map::InsertKeyFrame(Frame::Ptr frame) {
        keyframes_[frame->GetId()] = frame;
        cout << "The keyframe size is " << keyframes_

    }

    void InsertMapPoint(MapPoint::Ptr map_points);

}

