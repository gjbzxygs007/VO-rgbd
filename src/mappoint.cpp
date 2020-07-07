//
// Define the MapPoint class method
//

#include "slam_rgbd/mappoint.h"

namespace slamrgbd {
    MapPoint::Ptr MapPoint::CreateMapPoint() {
        return MapPoint::Ptr(new MapPoint(factory_id_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }

    MapPoint::Ptr MapPoint::CreateMapPoint(const Vector3d & pos_world, const Vector3d & norm, const Mat & descriptor, Frame::Ptr frame) {
        return MapPoint::Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
    }

    unsigned long MapPoint::factory_id_ = 0;
}

