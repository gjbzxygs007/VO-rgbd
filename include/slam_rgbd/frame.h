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
        Frame() : id_(-1), time_stamp_(-1), camera_(NULL) {}
        Frame(long id, double time_stamp = 0, SE3 transform_matrix_c_w = SE3(), Camera::Ptr camera = NULL, Mat color = Mat(), Mat depth = Mat());
        ~Frame() = default;
        double FindDepth(const cv::KeyPoint & key_point);
        Vector3d GetCameraCenter() const;
        bool IsInFrame(const Vector3d & point_w);

        static Frame::Ptr CreateFrame();
    };

    class MapPoint;
}
#endif //SLAMRGBD_FRAME_H
