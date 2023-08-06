#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
// #include <ros/ros.h>
#include "sensor_msgs/msg/imu.hpp"
// #include <sensor_msgs/Imu.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include "gyr_int.h"
#include "sophus/se3.hpp"
#include <cmath>
#include <time.h>
#include "localization_tools/condition_judge.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

struct MeasureGroup {
  sensor_msgs::msg::PointCloud2::ConstPtr lidar;
  // sensor_msgs::PointCloud2ConstPtr lidar;
  std::vector<sensor_msgs::msg::Imu::ConstPtr> imu;
};

class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas);
  void Reset();

  void IntegrateGyr(const std::vector<sensor_msgs::msg::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out, double dt_be,
                    const Sophus::SE3d &Tbe);
  void set_T_i_l(Eigen::Quaterniond& q, Eigen::Vector3d& t){
    T_i_l = Sophus::SE3d(q, t);
  }

  // ros::NodeHandle nh;
  rclcpp::Node::SharedPtr ros2_node_ = nullptr;

 private:
  /// Whether is the first frame, init for first frame
  bool b_first_frame_ = true;

  //// Input pointcloud
  PointCloudXYZI::Ptr cur_pcl_in_;
  //// Undistorted pointcloud
  PointCloudXYZI::Ptr cur_pcl_un_;

  double dt_l_c_;

  /// Transform form lidar to imu
  Sophus::SE3d T_i_l;
  //// For timestamp usage
  sensor_msgs::msg::PointCloud2::ConstPtr last_lidar_;
  sensor_msgs::msg::Imu::ConstPtr last_imu_;
  
  /// For gyroscope integration
  GyrInt gyr_int_;
};

#endif  // LOAM_HORIZON_DATA_PROCESS_H
