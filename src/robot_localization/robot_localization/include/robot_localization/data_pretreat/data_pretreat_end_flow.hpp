/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等,也是对应感知的接口
 */
#pragma once

#ifndef DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include "rclcpp/rclcpp.hpp"
// subscriber
#include "../subscriber/cloud_subscriber/cloud_subscriber_interface.hpp"
#include "../subscriber/cloud_subscriber/livox_cloud_subscriber.hpp"
#include "../subscriber/cloud_subscriber/standard_cloud_subscriber.hpp"
#include "../subscriber/imu_subscriber.hpp"
#include "../subscriber/cloud_subscriber.hpp"

#include "../tf/tf_listener.hpp"

// publisher
// a. synced lidar measurement
#include "../publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "../publisher/imu_publisher.hpp"
#include "../publisher/odometry_publisher.hpp"

#include "../subscriber/odometry_subscriber.hpp"
// 去畸变处理
#include "localization_models/scan_distortion_adjust/distortion_adjust.hpp"
#include "localization_models/scan_distortion_adjust/livox_cloud_undistortion/livox_dedistortion.hpp"

namespace robot_localization {
class DataPretreatFlow 
{
  public:
    DataPretreatFlow(std::shared_ptr<rclcpp::Node>& node_);
    bool Run();
    void LivoxDedistortion_thread_start();
  	static void LivoxDedistortion_thread_working(DataPretreatFlow*);
    rclcpp::Node::SharedPtr ros2_node = nullptr;
  private:
    bool ReadData();
    bool InitCalibration();

    bool HasData();
    bool ValidData();
    bool lidar_dedistortion();
    bool PublishData();
    bool lidar_cloud_sift();

  private:
    // subscriber
    // a. lidar odometry:
    std::shared_ptr<OdometrySubscriber> fused_odom_sub_ptr_;
    std::deque<PoseData> fused_odom_data_buff_;
    PoseData current_fused_odom_data_;

    std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
    
    std::shared_ptr<CloudSubscriberInterface> cloud_sub_ptr_;

    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;
    std::shared_ptr<LivoxDedistortion> livox_dedistortion_ptr_;
    Eigen::Matrix4d lidar_to_imu_ = Eigen::Matrix4d::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<ImuData> imu_data_buff_;

    CloudData current_cloud_data_;
    ImuData current_imu_data_;


};
}

#endif