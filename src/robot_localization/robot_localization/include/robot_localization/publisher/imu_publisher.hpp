/*
 * @Description: 在ros中发布IMU数据
 */
#ifndef PUBLISHER_IMU_PUBLISHER_HPP_
#define PUBLISHER_IMU_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_data/imu_data.hpp"

namespace robot_localization {
class IMUPublisher {
  public:
    inline IMUPublisher(
      std::shared_ptr<rclcpp::Node> &node_,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );
    IMUPublisher() = default;

    inline void Publish(const ImuData &imu_data, double time);
    inline void Publish(const ImuData &imu_data);
    inline void Publish(const ImuData &imu_data, builtin_interfaces::msg::Time time);

    inline bool HasSubscribers(void);

  private:
    inline void PublishData(const ImuData &imu_data, builtin_interfaces::msg::Time time);
    rclcpp::Node::SharedPtr ros2_node_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    // ros::NodeHandle nh_;
    // ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::msg::Imu imu_;
};

  IMUPublisher::IMUPublisher(
                              std::shared_ptr<rclcpp::Node> &node_,
                              std::string topic_name,
                              std::string frame_id,
                              size_t buff_size
                            ) :ros2_node_(node_), frame_id_(frame_id) 
    {
        publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_name,buff_size);
        imu_.header.frame_id = frame_id_;

    }

    void IMUPublisher::Publish(const ImuData &imu_data, double time) 
    {   
        time = time*1;
        builtin_interfaces::msg::Time ros2_time = ros2_node_->now();

        PublishData(imu_data, ros2_time);
    }

    void IMUPublisher::Publish(const ImuData &imu_data, builtin_interfaces::msg::Time time) 
    {
        PublishData(imu_data, time);
    }

    void IMUPublisher::Publish(const ImuData &imu_data) 
    {
        builtin_interfaces::msg::Time time = ros2_node_->now();
        PublishData(imu_data, time);
    }

    void IMUPublisher::PublishData(const ImuData &imu_data, builtin_interfaces::msg::Time time) 
    {
        imu_.header.stamp = time;
        
        // set orientation:
        imu_.orientation.w = imu_data.orientation_.w;
        imu_.orientation.x = imu_data.orientation_.x;
        imu_.orientation.y = imu_data.orientation_.y;
        imu_.orientation.z = imu_data.orientation_.z;

        // set angular velocity:
        imu_.angular_velocity.x = imu_data.angular_velocity_.x;
        imu_.angular_velocity.y = imu_data.angular_velocity_.y;
        imu_.angular_velocity.z = imu_data.angular_velocity_.z;

        // set linear acceleration:
        imu_.linear_acceleration.x = imu_data.linear_acceleration_.x;
        imu_.linear_acceleration.y = imu_data.linear_acceleration_.y;
        imu_.linear_acceleration.z = imu_data.linear_acceleration_.z;

        imu_.header.frame_id = frame_id_;
        publisher_->publish(imu_);
    }

    bool IMUPublisher::HasSubscribers(void) 
    {

        return (publisher_->get_subscription_count()!=0);
    }
} 
#endif