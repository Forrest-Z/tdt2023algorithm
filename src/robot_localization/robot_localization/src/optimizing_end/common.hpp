#pragma once
#ifndef OPTIMIZING_END_COMMON_HPP_
#define OPTIMIZING_END_COMMON_HPP_
#include <cmath>

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};
#endif