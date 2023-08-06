#ifndef __NAVIGATION_USART_SENDER_H
#define __NAVIGATION_USART_SENDER_H

#include <roborts_utils/base_blackboard.h>

#include <boost/asio.hpp>
#include <cstdint>
#include <rclcpp/qos.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "navigation_interface/msg/navigation2_usart.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "usart_black_board.h"

namespace tdtusart {
class NavigationUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ =
        node->create_subscription<navigation_interface::msg::Navigation2Usart>(
            "navigation2usart",       // topic_name,
            rclcpp::SensorDataQoS(),  // QOS,
            std::bind(&NavigationUsartSender::Callback, this,
                      std::placeholders::_1));
    usartSend_ = usartSend;
    tdttoolkit::BaseBlackboard::Init("NavMoveFlag", sizeof(bool));
    tdttoolkit::BaseBlackboard::Set("NavMoveFlag", false);
  }
#pragma pack(1)
  struct NavigationData {
    uint8_t header = 0xA5;
    uint8_t type = 1;
    double target_x_speed = 5;
    double target_y_speed = 6;
    uint8_t spin : 1;
    uint8_t lifting : 1;
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
    NavigationData send_data;
    send_data.target_x_speed = 0 - msg.x_speed;
    send_data.target_y_speed = msg.y_speed;
    // send_data.target_x_speed = 250;
    // send_data.target_y_speed = 250;
    send_data.spin = msg.spin;
    send_data.lifting = msg.lifting;
    send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    if (send_data.target_x_speed == 0 && send_data.target_y_speed == 0) {
      tdttoolkit::BaseBlackboard::Set("NavMoveFlag", false);
    } else {
      tdttoolkit::BaseBlackboard::Set("NavMoveFlag", true);
    }
    if (frame_id % 25 == 0) {
      auto angle = atan2(send_data.target_y_speed, send_data.target_x_speed);
      TDT_INFO("Send Navigation Data %f %f, angle=%f", send_data.target_x_speed,
               send_data.target_y_speed, angle);
    }
  }
};

}  // namespace tdtusart
#endif