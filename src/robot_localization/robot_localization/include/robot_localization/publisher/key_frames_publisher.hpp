/*
 * @Description: key frames 信息发布
 */
#ifndef LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <nav_msgs/Path.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/PoseStamped.h>

#include "sensor_data/key_frame.hpp"
#include "localization_tools/timestamp_transform.hpp"
namespace robot_localization {
class KeyFramesPublisher {
  public:
    KeyFramesPublisher(std::shared_ptr<rclcpp::Node> &node_, 
                       std::string topic_name, 
                       std::string frame_id,
                       int buff_size) : frame_id_(frame_id),ros2_node_(node_)
    {
        publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_name,buff_size);

    }
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames)
    {
        nav_msgs::msg::Path path;
        // path.header.stamp = ros::Time::now();
        path.header.stamp = ros2_node_->now();

        path.header.frame_id = frame_id_;

        for (size_t i = 0; i < key_frames.size(); ++i) {
            KeyFrame key_frame = key_frames.at(i);
            geometry_msgs::msg::PoseStamped pose_stamped;
            // geometry_msgs::PoseStamped pose_stamped;
            builtin_interfaces::msg::Time ros2_time = double_to_time(key_frame.time);
            // ros::Time ros_time(key_frame.time);
            // pose_stamped.header.stamp = ros_time;
            pose_stamped.header.stamp = ros2_time;
            pose_stamped.header.frame_id = frame_id_;

            
            // pose_stamped.header.seq = key_frame.index;//序列号不能使用，后面确定影响

            pose_stamped.pose.position.x = key_frame.pose(0,3);
            pose_stamped.pose.position.y = key_frame.pose(1,3);
            pose_stamped.pose.position.z = key_frame.pose(2,3);

            Eigen::Quaternionf q = key_frame.GetQuaternion();
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path.poses.push_back(pose_stamped);
        }

        publisher_->publish(path);
    }

    bool HasSubscribers()
    {
      return (publisher_->get_subscription_count()!=0);
    }

  private:
    rclcpp::Node::SharedPtr ros2_node_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    // ros::NodeHandle nh_;
    // ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif