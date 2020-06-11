// Frame and MapPoint class

#ifndef SLAMRGBD_FRAME_H
#define SLAMRGBD_FRAME_H
#include "slam_rgbd/common_include.h"
#include "slam_rgbd/camera.h"

namespace slamrgbd {
    class Frame {
    private:
        unsigned long id_;
        double time_stamp_;
        SE3 transform_matrix_c_w_;
        Camera::Ptr camera_;
        Mat color_, depth_;

    public:
        typedef std::shared_ptr <Frame> Ptr;
        Frame() = default;
        Frame(long id, double time_stamp = 0, SE3 transform_matrix_c_w = SE3(), Camera::Ptr camera = NULL, Mat color = Mat(), Mat depth = Mat());
        ~Frame();
        double FindDepth(const cv::KeyPoint & key_point);
        Vector3d GetCameraCenter() const;
        bool IsInFrame(const Vector3d & point_w);

        statoc Frame::Ptr CreateFrame();
    };

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
        MapPoint() = default;
        MapPoint(long id, Vector3d position, Vector3d norm);

        static MapPoint::Ptr CreateMapPoint();
    };


}
#endif //SLAMRGBD_FRAME_H
