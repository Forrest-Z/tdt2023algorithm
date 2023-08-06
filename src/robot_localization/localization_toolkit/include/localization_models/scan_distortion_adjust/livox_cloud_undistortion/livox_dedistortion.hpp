/*
 * @Description: livox给出的自运动畸变去除基于ros1，更改了接口升级到ros2,其中包含了时间戳的处理，因此在
    调用这部分的时候不处理时间戳，而是开个进程来跑，运行ProcessLoop(std::shared_ptr<rclcpp::Node>& node_)
 */

#ifndef MODELS_SCAN_ADJUST_LIVOX_DEDISTORTION_HPP_
#define MODELS_SCAN_ADJUST_LIVOX_DEDISTORTION_HPP_

#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>
#include "data_process.h"

class LivoxDedistortion
{
    private:
    
        // To notify new data
        std::mutex mtx_buffer;
        std::condition_variable sig_buffer;
        bool b_exit = false;
        bool b_reset = false;

        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> sub_pcl = nullptr;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> sub_imu = nullptr;
        /// Buffers for measurements
        double last_timestamp_lidar = -1;
        std::deque<sensor_msgs::msg::PointCloud2::ConstPtr> lidar_buffer;
        double last_timestamp_imu = -1;
        std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer;      
    public:
        void pointcloud_cbk(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
        {
                    // const double timestamp = msg->header.stamp.toSec();

            const double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec* 1e-9;

            // ROS_DEBUG("get point cloud at time: %.6f", timestamp);
            mtx_buffer.lock();
            if (timestamp < last_timestamp_lidar) 
            {
                std::cout<< "\033[1m\033[31m" << "lidar loop back, clear buffer"<< "\033[1m\033[31m"<< std::endl;

                // ROS_ERROR("lidar loop back, clear buffer");
                lidar_buffer.clear();
            }
            last_timestamp_lidar = timestamp;
            lidar_buffer.push_back(msg);
            std::cout << "received point size: " << float(msg->data.size())/float(msg->point_step) << "\n";
            mtx_buffer.unlock();

            sig_buffer.notify_all();
        }
        void imu_cbk(const sensor_msgs::msg::Imu::ConstPtr &msg_in)
        {
            sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));

            // double timestamp = msg->header.stamp.toSec();
            double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec* 1e-9;
            // ROS_DEBUG("get imu at time: %.6f", timestamp);

            mtx_buffer.lock();

            if (timestamp < last_timestamp_imu) 
            {
                std::cout<< "\033[1m\033[31m" << "imu loop back, clear buffer" << "\033[1m\033[31m"<< std::endl;
                // ROS_ERROR("imu loop back, clear buffer");
                imu_buffer.clear();
                b_reset = true;
            }
            last_timestamp_imu = timestamp;

            imu_buffer.push_back(msg);

            mtx_buffer.unlock();
            sig_buffer.notify_all();
        }
        bool SyncMeasure(MeasureGroup &measgroup) 
        {
            if (lidar_buffer.empty() || imu_buffer.empty()) 
            {
                /// Note: this will happen
                return false;
            }

            if ( (imu_buffer.front()->header.stamp.sec+imu_buffer.front()->header.stamp.nanosec* 1e-9) >
                (lidar_buffer.back()->header.stamp.sec+lidar_buffer.back()->header.stamp.nanosec* 1e-9)
                ) 
            {
                lidar_buffer.clear();
                std::cout<< "\033[1m\033[31m" << "clear lidar buffer, only happen at the beginning" << "\033[1m\033[31m"<< std::endl;

                // ROS_ERROR("clear lidar buffer, only happen at the beginning");
                return false;
            }

            if ((imu_buffer.back()->header.stamp.sec + imu_buffer.back()->header.stamp.nanosec* 1e-9)  <
                (lidar_buffer.front()->header.stamp.sec + lidar_buffer.front()->header.stamp.nanosec* 1e-9)
                ) 
            {
                return false;
            }

            /// Add lidar data, and pop from buffer
            measgroup.lidar = lidar_buffer.front();
            lidar_buffer.pop_front();
            // double lidar_time = measgroup.lidar->header.stamp.toSec();
            double lidar_time = measgroup.lidar->header.stamp.sec + measgroup.lidar->header.stamp.nanosec* 1e-9;

            /// Add imu data, and pop from buffer
            measgroup.imu.clear();
            int imu_cnt = 0;
            for (const auto &imu : imu_buffer) 
            {
                // double imu_time = imu->header.stamp.toSec();
                double imu_time = imu->header.stamp.sec + imu->header.stamp.nanosec* 1e-9;

                if (imu_time <= lidar_time) 
                {
                    measgroup.imu.push_back(imu);
                    imu_cnt++;
                }
            }
            for (int i = 0; i < imu_cnt; ++i) 
            {
                imu_buffer.pop_front();
            }
            // ROS_DEBUG("add %d imu msg", imu_cnt);

            return true;
        }

        /**
         * @brief 关键执行函数
         * @param ros2_node_ 调用这个部分的上一级ros2节点的指针(由上一级执行spin)
         **/
        void ProcessLoop(std::shared_ptr<rclcpp::Node>& node_) 
        {   
            std::shared_ptr<ImuProcess> p_imu(new ImuProcess());
            std::vector<double> vec;
            
            if(node_->get_parameter("/ExtIL",vec))
            {
                Eigen::Quaternion<double> q_il;
                Eigen::Vector3d t_il;
                q_il.w() = vec[0];
                q_il.x() = vec[1];
                q_il.y() = vec[2];
                q_il.z() = vec[3];
                t_il << vec[4], vec[5], vec[6];
                p_imu->set_T_i_l(q_il, t_il);
                RCLCPP_INFO(node_->get_logger(),"Extrinsic Parameter RESET ... ");
            }
            
            p_imu->ros2_node_ = node_;

            std::cout<<"Start ProcessLoop"<<std::endl;
            // ROS_INFO("Start ProcessLoop");
            // rclcpp::Rate r(1000);
            
            while (rclcpp::ok()) 
            {   

                MeasureGroup meas;
                std::unique_lock<std::mutex> lk(mtx_buffer);
                sig_buffer.wait(lk, [&meas,this]() -> bool { return SyncMeasure(meas) || b_exit; });
                lk.unlock();

                if (b_exit) 
                {
                    std::cout<<"b_exit=true, exit"<<std::endl;
                    // ROS_INFO("b_exit=true, exit");
                    break;
                }

                if (b_reset) 
                {   
                    std::cout<< "\033[1m\033[33m" << "reset when rosbag play back"<< "\033[1m\033[33m"<< std::endl;
                    // ROS_WARN("reset when rosbag play back");
                    p_imu->Reset();
                    b_reset = false;
                    continue;
                }
                p_imu->Process(meas);
                // r.sleep();
            }
        }
        void ProcessOnce();

        /**
         * @brief 默认构造，只在这里创建话题subscriber
         * @param ros2_node_ 调用这个部分的上一级ros2节点的指针(由上一级执行spin)
         * @param topic_pcl_name 输入的点云话题，毕竟是livox雷达，默认/livox/lidar
         * @param topic_imu_name 输入的IMU话题，/livox/imu
         **/
        LivoxDedistortion(  
                            rclcpp::Node::SharedPtr ros2_node_,
                            std::string topic_pcl_name,
                            std::string topic_imu_name
                         )
        {   

            // signal(SIGINT, SigHandle); //快速关闭函数
            
            sub_pcl = ros2_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                                topic_pcl_name,
                                1,
                                std::bind(&LivoxDedistortion::pointcloud_cbk, this, std::placeholders::_1)
            );
            
            sub_imu = ros2_node_->create_subscription<sensor_msgs::msg::Imu>(
                                topic_imu_name,
                                100,
                                std::bind(&LivoxDedistortion::imu_cbk, this, std::placeholders::_1)
            );

        }



     
};


#endif