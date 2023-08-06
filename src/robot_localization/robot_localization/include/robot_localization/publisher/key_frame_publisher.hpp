/*
 * @Description: 单个 key frame 信息发布
 */
#ifndef LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "sensor_data/key_frame.hpp"
#include "localization_tools/timestamp_transform.hpp"
namespace robot_localization {
class KeyFramePublisher {
  public:
    KeyFramePublisher(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, 
                      std::string frame_id,int buff_size): frame_id_(frame_id)
    {
      publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name,buff_size);

    }
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame)
    {
      // geometry_msgs::PoseWithCovarianceStamped pose_stamped;
      geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
      // ros::Time ros_time(key_frame.time);
      // pose_stamped.header.stamp = ros_time;
      pose_stamped.header.stamp = double_to_time(key_frame.time);

      pose_stamped.header.frame_id = frame_id_;

      pose_stamped.pose.pose.position.x = key_frame.pose(0,3);
      pose_stamped.pose.pose.position.y = key_frame.pose(1,3);
      pose_stamped.pose.pose.position.z = key_frame.pose(2,3);

      Eigen::Quaternionf q = key_frame.GetQuaternion();
      pose_stamped.pose.pose.orientation.x = q.x();
      pose_stamped.pose.pose.orientation.y = q.y();
      pose_stamped.pose.pose.orientation.z = q.z();
      pose_stamped.pose.pose.orientation.w = q.w();

      pose_stamped.pose.covariance[0] = (double)key_frame.index;

      publisher_->publish(pose_stamped);
    }

    bool HasSubscribers()
    {
      return (publisher_->get_subscription_count()!=0);
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    
    
    std::string frame_id_ = "";
};
}
#endif