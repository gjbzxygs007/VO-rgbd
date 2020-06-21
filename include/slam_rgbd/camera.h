//
// It defines the transformation between pixel, camera and world frame.
//

#ifndef SLAMRGBD_CAMERA_H
#define SLAMRGBD_CAMERA_H
#include "slam_rgbd/common_include.h"

namespace slamrgbd {
    class Camera {
    private:
        float fx_, fy_, cx_, cy_, depth_scale_; //camera intrinsics
    public:
        typedef shared_ptr<Camera> Ptr;
        Camera();
        Camera(float fx, float fy, float cx, float cy, float depth_scale = 0) : fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale ) {}
        // Transformation between coordinate systems
        inline float GetDepthScale() {return depth_scale_; }
        inline float GetFx() {return fx_; }
        inline float GetFy() {return fy_; }
        inline float GetCx() {return cx_; }
        inline float GetCy() {return cy_; }

        Vector3d WorldToCamera(const Vector3d & point_w, const SE3 & transform_matrix_c_w);
        Vector3d CameraToWorld(const Vector3d & point_c, const SE3 & transform_matrix_c_w);
        Vector2d CameraToPixel(const Vector3d & point_c);
        Vector3d PixelToCamera(const Vector2d & point_p, double depth = 1.0);
        Vector3d PixelToWorld(const Vector2d & point_p, const SE3 & transform_matrix_c_w, double depth = 1.0);
        Vector2d WorldToPixel(const Vector3d & point_w, const SE3 & transform_matrix_c_w);
    };

}


#endif //SLAMRGBD_CAMERA_H
