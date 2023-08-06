#ifndef __NAVIGATION_USART_SENDER_H
#define __NAVIGATION_USART_SENDER_H

#include <roborts_utils/base_blackboard.h>

#include <boost/asio.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "navigation_interface/msg/navigation2_usart.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "usart_black_board.h"

namespace tdtusart {
class NavigationPathSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ =
        node->create_subscription<navigation_interface::msg::Navigation2Usart>(
            "navigation_path2usart",  // topic_name,
            10,                       // buff_size,
            std::bind(&NavigationPathSender::Callback, this,
                      std::placeholders::_1));
    usartSend_ = usartSend;
  }
#pragma pack(1)
  struct NavigationPathData {
    uint8_t header = 0xA5;
    uint8_t type = 4;
    double path_x[50];
    double path_y[50];
    double time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // tdttoolkit::BaseCommunicator *communicator;
  rclcpp::Subscription<navigation_interface::msg::Navigation2Usart>::SharedPtr
      subscriber_;

  std::function<bool(const void *, int)> usartSend_;

  int frame_id = 0;

  void Callback(const navigation_interface::msg::Navigation2Usart &msg) {
    tdttoolkit::MatchInfo match_info;
    tdttoolkit::BaseBlackboard::Get("match_info", match_info);
    if (match_info.match_time < 0 && match_info.match_time > -90) {
      TDT_WARNING("match is not start.");
      return;
    }
    if (!usartblackboard::localization_start_ && match_info.match_time > -90) {
      TDT_WARNING("localization is not start.");
      return;
    }
    NavigationPathData send_data;
  }
};

}  // namespace tdtusart
#endif