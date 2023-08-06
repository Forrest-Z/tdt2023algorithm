#ifndef NODE_DEBUG_H
#define NODE_DEBUG_H

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vision_interface/msg/visiondebug_info.hpp"
#include "vision_interface/msg/group_msgs.hpp"

#include "vision_interface/srv/request_vision_param.hpp"
#include "vision_interface/srv/apply_vision_param.hpp"
#include "vision_interface/msg/image_compressed.hpp"
#include <cstring>
#include <deque>
#include <rclcpp/subscription_base.hpp>
#include <iostream>
#include "roborts_utils/debug.h"

// #include <yaml-cpp/yaml.h>

namespace node_debug
{   
    //ZLL-TODO: 1.现在的通讯方式基于ROS,后续考虑直接基于共享内存操作。现在这种方式会导致延迟较大。（涉及到两次拷贝构造方式）
    //另外此方案是以当前帧图像识别为基准，意味着如果其他模块处理必须在下一帧识别之前完成，否则会丢失那部分的信息。
    class NodeDebug
    {
        public:
            NodeDebug(rclcpp::Node::SharedPtr &predict_node);

        rclcpp::Node::SharedPtr debug_node_ = NULL ; //debug中间节点

        /*
        @brief: 用于匹配相同帧序号信息后发布视觉调试中间信息给前端qt
        @param[in]: seq: 帧序号
        */
        void matching_publish(int delay=1) ;  //匹配信息然后进行发布


        private:// 订阅话题

        rclcpp::Publisher<vision_interface::msg::VisiondebugInfo>::SharedPtr debug_pub_ = NULL; //发布视觉调试中间信息给前端qt

        rclcpp::Subscription<vision_interface::msg::ImageCompressed>::SharedPtr image_sub_ = NULL; //订阅图像信息

        rclcpp::Subscription<vision_interface::msg::GroupMsgs>::SharedPtr group_sub_ = NULL; //订阅group信息

        rclcpp::Service<vision_interface::srv::RequestVisionParam>::SharedPtr param_request = NULL; //debug申请参数服务

        rclcpp::Service<vision_interface::srv::ApplyVisionParam>::SharedPtr param_apply = NULL; //debug应用参数服务


        private: // 回调函数
        void ImageCallback(const vision_interface::msg::ImageCompressed::SharedPtr msg); //图像信息回调函数

        void GroupCallback(const vision_interface::msg::GroupMsgs::SharedPtr msg); //group信息回调函数

        void ParamRequestCallback(const vision_interface::srv::RequestVisionParam::Request::SharedPtr request,
                                  const vision_interface::srv::RequestVisionParam::Response::SharedPtr response); //参数申请回调函数
        
        void ParamApplyCallback(const vision_interface::srv::ApplyVisionParam::Request::SharedPtr request,
                                    const vision_interface::srv::ApplyVisionParam::Response::SharedPtr response); //参数应用回调函数


        private: // 一些数据处理的私有变量

        std::deque <vision_interface::msg::GroupMsgs> group_msgs_deque_; //group信息队列

        std::deque <vision_interface::msg::ImageCompressed> image_deque_; //图像信息队列

        unsigned int frame_seq = 0 ; //当前帧序号，一次为基准

        const int image_max_deque_size = 10 ; //图像队列最大长度
        const int group_max_deque_size = 4*image_max_deque_size; //消息队列最大长度

    };
}



#endif
