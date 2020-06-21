//
// The definition of camera class
//
#include "slam_rgbd/camera.h"

namespace slamrgbd {

    Vector3d Camera::WorldToCamera(const Vector3d & point_w, const SE3 & transform_matrix_c_w) {
        return transform_matrix_c_w * point_w;
    }

    Vector3d Camera::CameraToWorld(const Vector3d & point_c, const SE3 & transform_matrix_c_w) {
        return transform_matrix_c_w.inverse() * point_c;
    }

    Vector2d Camera::CameraToPixel(const Vector3d & point_c) {
        Vector2d point_p(fx_ * point_c(0, 0) / point_c(2, 0) + cx_, fy_ * point_c(1, 0) / point_c(2, 0) + cy_);
        return point_p;
    }

    Vector3d Camera::PixelToCamera(const Vector2d & point_p, double depth) {
        Vector3d point_c((point_p(0, 0) - cx_) * depth / fx_, (point_p(1, 0) - cy_) * depth / fy_, depth);
        return point_c;
    }

    Vector3d Camera::PixelToWorld(const Vector2d & point_p, const SE3 & transform_matrix_c_w, double depth) {
        return CameraToWorld(PixelToCamera(point_p, depth), transform_matrix_c_w);
    }
    Vector2d Camera::WorldToPixel(const Vector3d & point_w, const SE3 & transform_matrix_c_w) {
        CameraToPixel(WorldToCamera(point_w, transform_matrix_c_w));
    }

}