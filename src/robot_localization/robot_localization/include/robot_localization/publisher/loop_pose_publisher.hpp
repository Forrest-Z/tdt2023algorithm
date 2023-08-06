/*
 * @Description: 发送闭环检测的相对位姿
 * @Author: 
 * @Date: 
 */
#ifndef PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_data/loop_frame_pose.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "sensor_data/loop_frame_pose.hpp"
#include "localization_tools/timestamp_transform.hpp"
namespace robot_localization {
class LoopPosePublisher {
  public:
    LoopPosePublisher(std::shared_ptr<rclcpp::Node> &node_,
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size): frame_id_(frame_id)
    {
        publisher_=node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name,buff_size);
    }


    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;

      builtin_interfaces::msg::Time ros2_time = double_to_time(loop_pose.time);
      // ros::Time ros_time((float)loop_pose.time);
      pose_stamped.header.stamp = ros2_time;
      // pose_stamped.header.stamp = ros_time;
      pose_stamped.header.frame_id = frame_id_;

      pose_stamped.pose.pose.position.x = loop_pose.pose(0,3);
      pose_stamped.pose.pose.position.y = loop_pose.pose(1,3);
      pose_stamped.pose.pose.position.z = loop_pose.pose(2,3);

      Eigen::Quaternionf q = loop_pose.GetQuaternion();
      pose_stamped.pose.pose.orientation.x = q.x();
      pose_stamped.pose.pose.orientation.y = q.y();
      pose_stamped.pose.pose.orientation.z = q.z();
      pose_stamped.pose.pose.orientation.w = q.w();

      pose_stamped.pose.covariance[0] = (double)loop_pose.index0;
      pose_stamped.pose.covariance[1] = (double)loop_pose.index1;
      publisher_->publish(pose_stamped);

    }

    bool HasSubscribers()
    {
        return (publisher_->get_subscription_count()!=0);

    }

  private:
    std::string frame_id_ = "";
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};
}
#endif