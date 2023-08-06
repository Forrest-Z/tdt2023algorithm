/*
 * @Description: tf广播
 */
// #include "../../include/tf/tf_broadcaster.hpp"
#include "robot_localization/tf/tf_broadcaster.hpp"
namespace robot_localization
{
    TFBroadcaster::TFBroadcaster(
                                 std::shared_ptr<rclcpp::Node>& node_,
                                 std::string frame_id, 
                                 std::string child_frame_id
                                 ) : ros2_node_(node_)
    {   
        transform_.header.frame_id = frame_id;
        transform_.child_frame_id = child_frame_id;
        // transform_.frame_id_ = frame_id;
        // transform_.child_frame_id_ = child_frame_id;
    }

    void TFBroadcaster::SendTransform(Eigen::Matrix4d pose, double time)
    {   

        int32_t trans_sec = (int32_t)time;
        uint32_t trans_nanosec = (time-trans_sec)* 1e+9;

        builtin_interfaces::msg::Time ros2_time;
        ros2_time.sec = trans_sec;ros2_time.nanosec = trans_nanosec;

        static tf2_ros::TransformBroadcaster broadcaster_(ros2_node_);

        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        // ros::Time ros_time(time);
        // transform_.stamp_ = ros_time;
        time = time*1;
        transform_.header.stamp = ros2_time;
        transform_.transform.rotation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.transform.translation.x = pose(0, 3);
        transform_.transform.translation.y = pose(1, 3);
        transform_.transform.translation.z = pose(2, 3);

        // transform_.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        // transform_.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        broadcaster_.sendTransform(transform_);
    }
    void TFBroadcaster::SendTransform(Eigen::Matrix4d pose, builtin_interfaces::msg::Time time)
    {   
        static tf2_ros::TransformBroadcaster broadcaster_(ros2_node_);

        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        // ros::Time ros_time(time);
        // transform_.stamp_ = ros_time;
        transform_.header.stamp = time;
        
        transform_.transform.rotation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.transform.translation.x = pose(0, 3);
        transform_.transform.translation.y = pose(1, 3);
        transform_.transform.translation.z = pose(2, 3);

        // transform_.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        // transform_.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        broadcaster_.sendTransform(transform_);
    }

} // namespace robot_localization
