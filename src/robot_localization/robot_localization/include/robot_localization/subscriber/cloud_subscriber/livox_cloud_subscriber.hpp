/*
 * @Description: 激光雷达数据订阅
 */
#ifndef LIVOX_CLOUD_SUBSCRIBER_HPP
#define LIVOX_CLOUD_SUBSCRIBER_HPP

#define Livox 1
// c++
#include <deque>
// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//自定义点云数据类型
#include "sensor_data/cloud_data.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "../cloud_subscriber/cloud_subscriber_interface.hpp"
#include "localization_tools/color_terminal.hpp"
#include "localization_tools/timestamp_transform.hpp"
#include "localization_tools/color_terminal.hpp"
#include <chrono>
#include <sys/statfs.h>
namespace robot_localization
{
    class LivoxCloudSubscriber: public CloudSubscriberInterface 
    {
    public:
        LivoxCloudSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size)
        {
            subscriber_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                        topic_name,
                        buff_size,
                        std::bind(&LivoxCloudSubscriber::MsgCallback, this, std::placeholders::_1)
                                                                     );
        }
        LivoxCloudSubscriber() = default;
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

    public:
        void MsgCallback(const livox_ros_driver2::msg::CustomMsg::ConstPtr &cloud_msg)
        {   
            // white_info("livox_ros_driver2::msg::CustomMsg input size: " + std::to_string(cloud_msg->point_num));

            buff_mutex_.lock();
            CloudData cloud_data;
            
            // ROS2的时间类型 builtin_interfaces::msg::Time 需要把秒和纳秒相加才能表示当前时间
            // 如果获取的是msg时间戳，恢复成完整表达如下
            // double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            // auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

            cloud_data.ros2_time = cloud_msg->header.stamp;
            cloud_data.time_stamp_ = cloud_msg->header.stamp.sec + cloud_msg->header.stamp.nanosec* 1e-9;
            if(!check_for_lost_init)
            {
                check_for_lost_init = true;
                time_stamp_check_for_lost = cloud_data.time_stamp_;
            }else
            {


                    
                if(cloud_data.time_stamp_>(time_stamp_check_for_lost+0.2))
                {
                    // std::cout<<"imu两帧之间超时0.2s，可能丢包\n",
                    auto now = std::chrono::system_clock::now();
                    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
                    char timestamp_str[100] = {0};
                    std::strftime(timestamp_str, sizeof(timestamp_str), "%H%M_%S", std::localtime(&now_time_t));
                    std::string temp_str =  std::string("livox lidar 信息两帧之间超时0.2s，可能丢包 at time:")+ timestamp_str ;
                    cyan_info(temp_str);
                    std::cout<<"front: sec"<< double_to_time(time_stamp_check_for_lost).sec<<"front: nanosec"<<double_to_time(time_stamp_check_for_lost).nanosec <<"\n";
                    std::cout<<"back: sec"<< cloud_data.ros2_time.sec<<"back: nanosec"<<cloud_data.ros2_time.nanosec <<"\n";
                }
                time_stamp_check_for_lost=cloud_data.time_stamp_;


            }
            static double blind = 2.0;
            pcl::PointCloud<pcl::PointXYZINormal> pl_full;
            pl_full.clear();
            pl_full.resize(cloud_msg->point_num);

            pcl::PointCloud<pcl::PointXYZINormal> pl_surf;
            pl_surf.clear();
            pl_surf.resize(cloud_msg->point_num);

            for(uint i=1; i< cloud_msg->point_num ; i++)
            {
                if((cloud_msg->points[i].line < 4) && ((cloud_msg->points[i].tag & 0x30) == 0x10 || (cloud_msg->points[i].tag & 0x30) == 0x00))
                {   

                    pl_full[i].x = cloud_msg->points[i].x;
                    pl_full[i].y = cloud_msg->points[i].y;
                    pl_full[i].z = cloud_msg->points[i].z;
                    pl_full[i].intensity = cloud_msg->points[i].reflectivity;
                    pl_full[i].curvature = cloud_msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

                    if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
                        || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                        || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
                        && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
                    {
                        //检查当前点与前一个点的位置差异，如果它们的距离足够大且距离LiDAR的距离大于盲区半径，则将当前点添加
                        // cloud_data.cloud_ptr->push_back(pl_full[i]);
                        pl_surf.push_back(pl_full[i]);
                    }
                    
                }
            }
            // white_info("livox_ros_driver2::msg::CustomMsg pl_surf size: "+ std::to_string(pl_surf.size())+"\n");

            *(cloud_data.cloud_ptr) = pl_surf;

            buff_mutex_.unlock();
            new_cloud_data_buff_.push_back(cloud_data);
            
        }

    private:
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscriber_;
        // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        std::mutex buff_mutex_; 
        double time_stamp_check_for_lost;
        bool check_for_lost_init= false;

        std::deque<CloudData> new_cloud_data_buff_;
    };

} // namespace robot_localization

#endif