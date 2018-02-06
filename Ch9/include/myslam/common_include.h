#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.h>

using Sophus::SE3;

#include <opencv2/core/core.hpp>

using cv::Mat;

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;

#endif
