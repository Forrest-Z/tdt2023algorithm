/*
 * @Description: 订阅 闭环检测位姿 数据
 * @Author: 
 * @Date: 
 */
#ifndef SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_
#define SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "sensor_data/loop_frame_pose.hpp"
#include "localization_tools/timestamp_transform.hpp"
namespace robot_localization {
class LoopPoseSubscriber {
  public:
    LoopPoseSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size)
    {
        subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                      topic_name,
                      buff_size,
                      std::bind(&LoopPoseSubscriber::msg_callback, this, std::placeholders::_1));   
    }
    
    LoopPoseSubscriber() = default;
    void ParseData(std::deque<LoopPose>& loop_pose_buff)
    {
      buff_mutex_.lock();
      if (new_loop_pose_.size() > 0) {
          loop_pose_buff.insert(loop_pose_buff.end(), new_loop_pose_.begin(), new_loop_pose_.end());
          new_loop_pose_.clear();
      }
      buff_mutex_.unlock();
    }

  private:
    void msg_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& loop_pose_msg)
    {
      buff_mutex_.lock();
      LoopPose loop_pose;
      loop_pose.time = time_to_double(loop_pose_msg.header.stamp); 
      loop_pose.index0 = (unsigned int)loop_pose_msg.pose.covariance[0];
      loop_pose.index1 = (unsigned int)loop_pose_msg.pose.covariance[1];

      loop_pose.pose(0,3) = loop_pose_msg.pose.pose.position.x;
      loop_pose.pose(1,3) = loop_pose_msg.pose.pose.position.y;
      loop_pose.pose(2,3) = loop_pose_msg.pose.pose.position.z;

      Eigen::Quaternionf q;
      q.x() = loop_pose_msg.pose.pose.orientation.x;
      q.y() = loop_pose_msg.pose.pose.orientation.y;
      q.z() = loop_pose_msg.pose.pose.orientation.z;
      q.w() = loop_pose_msg.pose.pose.orientation.w;
      loop_pose.pose.block<3,3>(0,0) = q.matrix();

      new_loop_pose_.push_back(loop_pose);
      buff_mutex_.unlock();
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;

    std::deque<LoopPose> new_loop_pose_;

    std::mutex buff_mutex_; 
};
}
#endif