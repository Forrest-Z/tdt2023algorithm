/*
 * @Description: 订阅 key frame 数据
 * @Author: 
 * @Date: 
 */
#ifndef LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "sensor_data/key_frame.hpp"
#include "localization_tools/timestamp_transform.hpp"

namespace robot_localization {
class KeyFrameSubscriber {
  public:
    KeyFrameSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size)
    {
      subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                        topic_name,
                        buff_size,
                        std::bind(&KeyFrameSubscriber::msg_callback, this, std::placeholders::_1));   
                        // nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback, this);

    }
    KeyFrameSubscriber() = default;
    
    void ParseData(std::deque<KeyFrame>& key_frame_buff)
    {
        buff_mutex_.lock();
        if (new_key_frame_.size() > 0) 
        {
            key_frame_buff.insert(key_frame_buff.end(), new_key_frame_.begin(), new_key_frame_.end());
            new_key_frame_.clear();
        }
        buff_mutex_.unlock();
    }

  private:
    void msg_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& key_frame_msg)
    {
      buff_mutex_.lock();
      KeyFrame key_frame;
      key_frame.time = time_to_double(key_frame_msg.header.stamp); 
      key_frame.index = (unsigned int)key_frame_msg.pose.covariance[0];

      key_frame.pose(0,3) = key_frame_msg.pose.pose.position.x;
      key_frame.pose(1,3) = key_frame_msg.pose.pose.position.y;
      key_frame.pose(2,3) = key_frame_msg.pose.pose.position.z;

      Eigen::Quaternionf q;
      q.x() = key_frame_msg.pose.pose.orientation.x;
      q.y() = key_frame_msg.pose.pose.orientation.y;
      q.z() = key_frame_msg.pose.pose.orientation.z;
      q.w() = key_frame_msg.pose.pose.orientation.w;
      key_frame.pose.block<3,3>(0,0) = q.matrix();

      new_key_frame_.push_back(key_frame);
      buff_mutex_.unlock();
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;

    std::deque<KeyFrame> new_key_frame_;

    std::mutex buff_mutex_; 
};
}
#endif