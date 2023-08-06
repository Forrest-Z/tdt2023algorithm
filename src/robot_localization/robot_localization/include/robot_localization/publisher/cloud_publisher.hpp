/*
 * @Description: 点云发布
 */
#ifndef PUBLISHER_CLOUD_PUBLISHER_HPP_
#define PUBLISHER_CLOUD_PUBLISHER_HPP_

// ros
#include "sensor_msgs/msg/point_cloud2.hpp"
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// sensor_data
#include "sensor_data/cloud_data.hpp"
#include "localization_tools/timestamp_transform.hpp"
namespace robot_localization
{
    class CloudPublisher
    {
    public:
        inline CloudPublisher(std::shared_ptr<rclcpp::Node> &node_,
                       std::string topic_name,
                       std::string frame_id,
                       size_t buff_size);
        CloudPublisher() = default;

        inline void Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time);
        inline void Publish(CloudData::CLOUD_PTR &cloud_ptr_input);
        inline void Publish(CloudData::CLOUD_PTR &cloud_ptr_input,builtin_interfaces::msg::Time time);
        inline void Publish(
                     pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_ptr_input,
                     builtin_interfaces::msg::Time time
                    )
        {
            PublishData(cloud_ptr_input, time);
        }
        
        inline bool HasSubscribers();

    private:
        inline void PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, builtin_interfaces::msg::Time time);
        inline void PublishData(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_ptr_input, builtin_interfaces::msg::Time time)
        {
            sensor_msgs::msg::PointCloud2 cloud_ptr_output;
            pcl::toROSMsg(*cloud_ptr_input, cloud_ptr_output);

            cloud_ptr_output.header.stamp = time;
            cloud_ptr_output.header.frame_id = frame_id_;

            publisher_->publish(cloud_ptr_output);
        }

        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        std::string frame_id_;
    };

    CloudPublisher::CloudPublisher(
                                    std::shared_ptr<rclcpp::Node> &node_,
                                    std::string topic_name,
                                    std::string frame_id,
                                    size_t buff_size
                                  )
        : ros2_node_(node_), frame_id_(frame_id)
    {   
        publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name,buff_size);

    }



    bool CloudPublisher::HasSubscribers()
    {
        // return publisher_.getNumSubscribers() != 0;
        // ros2 好像是这个函数

        return (publisher_->get_subscription_count()!=0);
    }
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time)
    {   
        
        // double 转换 builtin_interfaces::msg::Time手写了函数
        builtin_interfaces::msg::Time ros2_time = double_to_time(time);
        // std::chrono::seconds(time);
        // builtin_interfaces::msg::Time ros2_time =  rclcpp::Time(time);//测试，这个只能把double硬转为int类型的nanosec并令sec=0
        // ros::Time ros_time(time);

        PublishData(cloud_ptr_input, ros2_time);
    }

    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input,builtin_interfaces::msg::Time time)
    {
        
        PublishData(cloud_ptr_input, time);
    }
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input)
    {
        builtin_interfaces::msg::Time time = ros2_node_->now();
        PublishData(cloud_ptr_input, time);
    }

    void CloudPublisher::PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, builtin_interfaces::msg::Time time)
    {
        sensor_msgs::msg::PointCloud2 cloud_ptr_output;
        pcl::toROSMsg(*cloud_ptr_input, cloud_ptr_output);

        cloud_ptr_output.header.stamp = time;
        cloud_ptr_output.header.frame_id = frame_id_;

        publisher_->publish(cloud_ptr_output);
    }

    


} // namespace robot_localization

#endif