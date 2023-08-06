#ifndef __SPEED_USART_RECVER_H
#define __SPEED_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <navigation_interface/msg/detail/usart_speed2_nav__struct.hpp>
#include <vector>

#include "base_usart.h"
#include "navigation_interface/msg/usart_speed2_nav.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"

namespace tdtusart {
class SpaeedUsartRecver : public BaseUsartRecver {
 public:
  SpaeedUsartRecver(){};

  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    speed_pub =
        node->create_publisher<navigation_interface::msg::UsartSpeed2Nav>(
            "usart_speed2_nav", tdttoolkit::HighFrequencySensorDataQoS());
  }
#pragma pack(1)
  struct SpeedData {
    uint8_t header = 0xA5;
    uint8_t type = 5;

    int16_t speed_x = 0;
    int16_t speed_y = 0;

    float timeStamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(SpeedData); }

  void ParseData(void *message) override {
    SpeedData *vision_data = (SpeedData *)message;
    navigation_interface::msg::UsartSpeed2Nav speed_msg;
    speed_msg.header.frame_id = vision_data->frame_id;
    speed_msg.header.stamp =
        tdttoolkit::Time::GetRosTimeByTime(vision_data->timeStamp / 1e6);
    speed_msg.speed_x = vision_data->speed_x;
    speed_msg.speed_y = vision_data->speed_y;
    // TDT_DEBUG("Recv usart nav speed data: x=%d,y=%d at time=%f",
    // vision_data->speed_x, vision_data->speed_y,vision_data->timeStamp);
    speed_pub->publish(speed_msg);
  }

  rclcpp::Publisher<navigation_interface::msg::UsartSpeed2Nav>::SharedPtr
      speed_pub;
};
}  // namespace tdtusart

#endif
