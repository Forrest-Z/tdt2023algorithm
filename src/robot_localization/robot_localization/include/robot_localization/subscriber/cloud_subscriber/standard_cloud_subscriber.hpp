/*
 * @Description: 激光雷达数据订阅
 */
#ifndef STANDARD_CLOUD_SUBSCRIBER_HPP
#define STANDARD_CLOUD_SUBSCRIBER_HPP

// c++
#include <deque>
// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//自定义点云数据类型
#include "sensor_data/cloud_data.hpp"
#include "../cloud_subscriber/cloud_subscriber_interface.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace robot_localization
{
    class StandardCloudSubscriber: public CloudSubscriberInterface 
    {
    public:
        StandardCloudSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size)
        {
            subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_name,
                        buff_size,
                        std::bind(&StandardCloudSubscriber::MsgCallback, this, std::placeholders::_1)
                                                                     );
        }
        StandardCloudSubscriber() = default;
        void ParseData(std::deque<CloudData> &cloud_data_buff)
        {
            buff_mutex_.lock();

            if (new_cloud_data_buff_.size() > 0)
            {
            cloud_data_buff.insert(cloud_data_buff.end(),
                                   new_cloud_data_buff_.begin(), new_cloud_data_buff_.end());
            new_cloud_data_buff_.clear();
            }
            buff_mutex_.unlock();

        }

    private:
        void MsgCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg)
        {
            CloudData cloud_data;
            buff_mutex_.lock();
            // ROS2的时间类型 builtin_interfaces::msg::Time 需要把秒和纳秒相加才能表示当前时间
            // 如果获取的是msg时间戳，恢复成完整表达如下
            // double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            // auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            cloud_data.ros2_time = cloud_msg.header.stamp;
            cloud_data.time_stamp_ = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec* 1e-9;

            // cloud_data.time_stamp_ = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec* 1e-9;

            pcl::fromROSMsg(cloud_msg, *(cloud_data.cloud_ptr));

            new_cloud_data_buff_.push_back(cloud_data);
            buff_mutex_.unlock();

        }

    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        std::mutex buff_mutex_; 

        std::deque<CloudData> new_cloud_data_buff_;
    };

} // namespace robot_localization

#endif