#ifndef __MATCH_INFO_REPLY_USART_SENDER_H
#define __MATCH_INFO_REPLY_USART_SENDER_H

#include <boost/asio.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/int16.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "navigation_interface/msg/navigation2_usart.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {
class CommandReplyUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    usartSend_ = usartSend;
    subscriber_ = node->create_subscription<std_msgs::msg::Int16>(
        "usart_command_reply",  // topic_name,
        10,                     // buff_size,
        std::bind(&CommandReplyUsartSender::Callback, this,
                  std::placeholders::_1));
  }
#pragma pack(1)
  struct CommandReplyData {
    uint8_t header = 0xA5;
    uint8_t type = 3;
    int16_t reply_frame_id;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  std::function<bool(const void *, int)> usartSend_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_;

 private:
  void Callback(const std_msgs::msg::Int16 &msg) {
    CommandReplyData send_data;
    send_data.reply_frame_id = msg.data;
    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    // TDT_INFO("Send Command Reply Data %d", send_data.reply_frame_id);
  }

  int frame_id = 0;
};

}  // namespace tdtusart
#endif