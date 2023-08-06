/*
 * @Description: 点云接收模块接口
 */
#ifndef CLOUD_SUBSCRIBER_INTERFACE_HPP
#define CLOUD_SUBSCRIBER_INTERFACE_HPP

// c++
#include <deque>
// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//自定义点云数据类型
#include "sensor_data/cloud_data.hpp"

namespace robot_localization
{
    class CloudSubscriberInterface
    {
    public:
        virtual ~CloudSubscriberInterface() = default;
        virtual void ParseData(std::deque<CloudData> &cloud_data_buff) = 0;
        //= 0: 表示这是一个纯虚函数。纯虚函数在基类中没有具体实现，需要在派生类中实现。包含纯虚函数的类被称为抽象类，不能创建抽象类的对象

    private:
        // virtual void MsgCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg) = 0;

    };

} // namespace robot_localization

#endif