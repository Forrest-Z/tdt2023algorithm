#include <rclcpp/rclcpp.hpp>
#include "node_debug/node_debug.h"
#include "roborts_utils/config.h"
#include "roborts_utils/base_msg.h"

int main(int argc, char **argv) {

    tdtconfig::Init();

    if(tdtconfig::VIDEODEBUG){

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr debug_ptr = rclcpp::Node::make_shared("debug_node");

    rclcpp::WallRate loop_rate(120); // 循环频率为100Hz
    node_debug::NodeDebug node_debug(debug_ptr);

    while (rclcpp::ok()) {

        rclcpp::spin_some(node_debug.debug_node_);// 用于处理回调函数
        
        node_debug.matching_publish(1); // 匹配信息（上一帧）然后进行发布

        loop_rate.sleep();
    }

    node_debug.matching_publish(0); // 退出时发布一个最新的匹配信息
    rclcpp::shutdown();

}
else{
    TDT_INFO("未开启debug");
}

    return 0;
}