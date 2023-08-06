/*
 * @Description: 里程计发布
 */
#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

// ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
// c++
#include <string>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class OdometryPublisher
    {
    public:
        inline OdometryPublisher(std::shared_ptr<rclcpp::Node>& node_,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          size_t buff_size);
        OdometryPublisher() = default;

        inline void Publish(const Eigen::Matrix4d &transform_matrix, double time);
        inline void Publish(const Eigen::Matrix4d &transform_matrix);
        inline void Publish(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, double time);
        inline void Publish(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time);
        inline void Publish(const Eigen::Matrix4d &transform_matrix, 
                    const Eigen::Vector3d &vel,
                    builtin_interfaces::msg::Time time
                    );
        inline void Publish(const nav_msgs::msg::Odometry &odom_msg);

        inline bool HasSubscriber();

    private:
        inline void PublishData(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time);

        inline void PublishData( 
                          const Eigen::Matrix4d &transform_matrix, 
                          const Eigen::Vector3d &vel, 
                          builtin_interfaces::msg::Time time
                        );

    private:
        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        
        nav_msgs::msg::Odometry odometry_;
    };

    /**
     * @brief 里程计发布初始化
     * @note
     * @todo
     **/
    OdometryPublisher::OdometryPublisher(
                                         std::shared_ptr<rclcpp::Node>& node_,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         size_t buff_size
                                        ): ros2_node_(node_)
    {

        publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name,buff_size);

        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    /**
     * @param transform_matrix
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix)
    {
        PublishData(transform_matrix, ros2_node_->now());
    }

    /**
     * @param transform_matrix
     * @param time
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix, double time)
    {   
        builtin_interfaces::msg::Time ros2_time;
        int32_t trans_sec = (int32_t)time;
        uint32_t trans_nanosec = (time-trans_sec)* 1e+9;
        
        ros2_time.sec = trans_sec;ros2_time.nanosec = trans_nanosec;
        PublishData(transform_matrix, ros2_time);
    }

    /**
     * @param transform_matrix
     * @param builtin_interfaces::msg::Time time
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time)
    {
        PublishData(transform_matrix,time);
    }

    /**
     * @param transform_matrix
     * @param vel
     * @param time
     **/
    void OdometryPublisher::Publish(
                                    const Eigen::Matrix4d &transform_matrix, 
                                    const Eigen::Vector3d &vel, 
                                    double time
                                    )
    {   
        builtin_interfaces::msg::Time ros2_time;
        int32_t trans_sec = (int32_t)time;
        uint32_t trans_nanosec = (time-trans_sec)* 1e+9;    
        ros2_time.sec = trans_sec;ros2_time.nanosec = trans_nanosec;

        PublishData(transform_matrix, vel, ros2_time);
    }
    
    /**
     * @param transform_matrix
     * @param vel
     * @param builtin_interfaces::msg::Time
     **/
    void OdometryPublisher::Publish(
                                    const Eigen::Matrix4d &transform_matrix, 
                                    const Eigen::Vector3d &vel, 
                                    builtin_interfaces::msg::Time time
                                    )
    {
        PublishData(transform_matrix, vel, time);
    }

    /**
     * @param transform_matrix
     **/
    void OdometryPublisher::Publish(const nav_msgs::msg::Odometry &odom_msg)
    {
        publisher_->publish(odom_msg);
    }
    
    /**
     * @brief 里程数据发布
     * @note
     * @todo
     **/
    void OdometryPublisher::PublishData(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time)
    {
        odometry_.header.stamp = time;

        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaterniond q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        publisher_->publish(odometry_);
    }

    void OdometryPublisher::PublishData(
                                        const Eigen::Matrix4d &transform_matrix,
                                        const Eigen::Vector3d &vel,
                                        builtin_interfaces::msg::Time time
                                       )
    {
        odometry_.header.stamp = time;

        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaterniond q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();
        
        odometry_.twist.twist.linear.x = vel.x();
        odometry_.twist.twist.linear.y = vel.y();
        odometry_.twist.twist.linear.z = vel.z();

        publisher_->publish(odometry_);
    }

    /**
     * @brief 是否被订阅
     * @note
     * @todo
     **/
    bool OdometryPublisher::HasSubscriber()
    {
        // return publisher_.getNumSubscribers() != 0;
        // ros2 好像是这个函数

        return (publisher_->get_subscription_count()!=0);
    }
    
} // namespace robot_localization

#endif