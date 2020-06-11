//
// Created by ubuntu-jianan on 6/11/20.
//

#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

// std library
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;


#endif //MYSLAM_COMMON_INCLUDE_H
