//
// Define the MapPoint class method
//

#include "slam_rgbd/mappoint.h"

namespace slamrgbd {
    MapPoint::MapPoint(long id, Vector3d position, Vector3d norm)
    : id_(id), pos_(position), norm_(norm), observed_times_(0), correct_times_(0) {}

    MapPoint::Ptr MapPoint::CreateMapPoint() {
        static long factory_id = 0;
        return MapPoint::Ptr(new MapPoint(factory_id++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }
}

