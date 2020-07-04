//
// Created by ubuntu-jianan on 6/22/20.
//

#ifndef SLAMRGBD_G2O_TYPES_H
#define SLAMRGBD_G2O_TYPES_H

#include "slam_rgbd/common_include.h"
#include "slam_rgbd/camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace slamrgbd {
class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(istream & in) {};
    virtual bool write(ostream & out) {};
};

class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(istream & in) {}
    virtual bool write(ostream & out) const {}

private:
    Vector3d point_;
};

class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(istream & in) {}
    virtual bool write(ostream & out) const {}
    inline void SetCamera(const Camera::Ptr & cam) {
        camera_ = cam;
    }
    inline void SetPoint(double x, double y, double z) {
        point_ << x, y, z;
    }


private:
    Vector3d point_;
    Camera::Ptr camera_;
};


}

#endif //SLAMRGBD_G2O_TYPES_H
