/*
 * @Description: 订阅 key frames 数据
 */
#ifndef LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <nav_msgs/Path.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/PoseStamped.h>
#include "sensor_data/key_frame.hpp"
#include "localization_tools/timestamp_transform.hpp"

namespace robot_localization {
class KeyFramesSubscriber {
  public:
    KeyFramesSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size)
    {
        subscriber_ = node_->create_subscription<nav_msgs::msg::Path>(
                        topic_name,
                        buff_size,
                        std::bind(&KeyFramesSubscriber::msg_callback, this, std::placeholders::_1));  
        
    }
    KeyFramesSubscriber() = default;

    void ParseData(std::deque<KeyFrame>& key_frames_buff)
    {
        buff_mutex_.lock();
      if (new_key_frames_.size() > 0) 
      {
          key_frames_buff = new_key_frames_;
          new_key_frames_.clear();
      }
    buff_mutex_.unlock();
    }

  private:
    void msg_callback(const nav_msgs::msg::Path& key_frames_msg_ptr)
    {
        buff_mutex_.lock();   
        new_key_frames_.clear();

        for (size_t i = 0; i < key_frames_msg_ptr.poses.size(); i++) {
            KeyFrame key_frame;
            key_frame.time = time_to_double(key_frames_msg_ptr.poses.at(i).header.stamp);
            // key_frame.time = key_frames_msg_ptr.poses.at(i).header.stamp.toSec();
            key_frame.index = (unsigned int)i;

            key_frame.pose(0,3) = key_frames_msg_ptr.poses.at(i).pose.position.x;
            key_frame.pose(1,3) = key_frames_msg_ptr.poses.at(i).pose.position.y;
            key_frame.pose(2,3) = key_frames_msg_ptr.poses.at(i).pose.position.z;

            Eigen::Quaternionf q;
            q.x() = key_frames_msg_ptr.poses.at(i).pose.orientation.x;
            q.y() = key_frames_msg_ptr.poses.at(i).pose.orientation.y;
            q.z() = key_frames_msg_ptr.poses.at(i).pose.orientation.z;
            q.w() = key_frames_msg_ptr.poses.at(i).pose.orientation.w;
            key_frame.pose.block<3,3>(0,0) = q.matrix();

            new_key_frames_.push_back(key_frame);
        }
        buff_mutex_.unlock();
    }

  private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_;
    
    std::deque<KeyFrame> new_key_frames_;

    std::mutex buff_mutex_; 
};
}
#endif