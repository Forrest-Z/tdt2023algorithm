#ifndef __COMMANDER_USART_RECVER_H
#define __COMMANDER_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/int16.hpp>
#include <vector>

#include "base_usart.h"
#include "navigation_interface/msg/nav_command.hpp"
#include "roborts_utils/roborts_utils.h"

namespace tdtusart {
class CommanderUsartRecver : public BaseUsartRecver {
 public:
// clang-format off
#pragma pack(1)
  struct CommanderData { // 操作手每次触发指令发送一次，视觉收到后100ms内回复确认收到
    uint8_t header = 0xA5;
    uint8_t type = 2;
    
    uint8_t target_type = 0;  // 机器人目标类型 0: 其他 1.进攻a 2.防守b 3.警戒i
    float target_x = -1; // 机器人手动控制目标点，若发送负值则让机器人回到自动模式
    float target_y = -1; // 机器人手动控制目标点，若发送负值则让机器人回到自动模式
    uint8_t marked_id = 0; //云台手标记的机器人id

    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
// clang-format on
#pragma pack()

  int GetStructLength() override { return sizeof(CommanderData); }

  void ParseData(void *message) override {
    CommanderData *commander_data = (CommanderData *)message;
    std_msgs::msg::Int16 reply_msg;
    reply_msg.data = commander_data->frame_id;
    reply_publisher_->publish(reply_msg);
    if (commander_data->frame_id <= last_recv_frame_id &&
        last_recv_frame_id - commander_data->frame_id < 200) {
      return;
    }
    TDT_DEBUG("Recv Command manual_id=%d,target_x=%f,target_y=%f",
              commander_data->target_type, commander_data->target_x,
              commander_data->target_y);
    last_recv_frame_id = commander_data->frame_id;
    navigation_interface::msg::NavCommand nav_command_msg;
    if ((commander_data->target_x < 0 || commander_data->target_y < 0) &&
        commander_data->marked_id == 0) {
       nav_command_msg.marked_id = commander_data->marked_id;
    } else {
      nav_command_msg.manual_position_x = commander_data->target_x;
      nav_command_msg.manual_position_y = commander_data->target_y;
      nav_command_msg.target_type = commander_data->target_type;
    }
    nav_command_publisher_->publish(nav_command_msg);
  }

  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    reply_publisher_ =
        node->create_publisher<std_msgs::msg::Int16>("usart_command_reply", 10);
    nav_command_publisher_ =
        node->create_publisher<navigation_interface::msg::NavCommand>(
            "nav_command", 10);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr reply_publisher_;
  rclcpp::Publisher<navigation_interface::msg::NavCommand>::SharedPtr
      nav_command_publisher_;
  int last_recv_frame_id = -1;
};
}  // namespace tdtusart

#endif