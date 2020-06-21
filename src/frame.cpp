//
// The definition of Frame and MapPoint class members
//

#include "slam_rgbd/frame.h"

namespace slamrgbd {

    Frame::Frame(long id, double time_stamp, SE3 transform_matrix_c_w, Camera::Ptr camera,
                 Mat color, Mat depth)
            : id_(id), time_stamp_(time_stamp), transform_matrix_c_w_(transform_matrix_c_w), camera_(camera),
              color_(color), depth_(depth) {}

    double Frame::FindDepth(const cv::KeyPoint &key_point) {
        int x = cvRound(key_point.pt.x);
        int y = cvRound(key_point.pt.y);
        ushort d = depth_.ptr<ushort>(y)[x];
        if (d != 0) {
            return double(d) / camera_->GetDepthScale();
        } else {
            int dx[4] = {-1, 0, 1, 0};
            int dy[4] = {0, -1, 0, 1};
            for (int i = 0; i < 4; ++i) {
                d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
                if(d != 0)
                {
                    return double(d) / camera_->GetDepthScale();
                }
            }
        }
        return -1.0;
    }

    Vector3d Frame::GetCameraCenter() const {
        return transform_matrix_c_w_.inverse().translation();
    }

    bool Frame::IsInFrame(const Vector3d &point_w) {
        Vector3d point_c = camera_->WorldToCamera(point_w, transform_matrix_c_w_);
        if (point_c(2, 0) < 0) {
            return false;
        }
        Vector2d point_p = camera_->WorldToPixel(point_w, transform_matrix_c_w_);
        return point_p(0, 0) > 0 && point_p(1, 0) > 0 && point_p(0, 0) < color_.cols && point_p(1, 0) < color_.rows;
    }

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0;
        return Frame::Ptr(new Frame(factory_id++));
    }

}

