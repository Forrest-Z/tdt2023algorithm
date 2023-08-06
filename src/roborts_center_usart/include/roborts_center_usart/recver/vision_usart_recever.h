#ifndef __VISION_USART_RECVER_H
#define __VISION_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <vector>

#include "base_usart.h"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "vision_interface/msg/usart_pub.hpp"

namespace tdtusart {
class VisionUsartRecver : public BaseUsartRecver {
 public:
  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    usartPub = node->create_publisher<vision_interface::msg::UsartPub>(
        "visionUsartPub", 1);
  }
#pragma pack(1)
  struct VisionData {
    uint8_t header = 0xA5;
    uint8_t type = 3;

    float yaw;
    float pitch;
    uint8_t colorvincible;
    uint8_t enemycolor : 1;
    uint8_t lockcommand : 1;
    float nominal_bulletspeed;
    float bullet_speed;
    uint8_t beat_mode;
    uint8_t buff_type;

    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(VisionData); }

  void ParseData(void *message) override {
    VisionData *vision_data = (VisionData *)message;

    vision_interface::msg::UsartPub usrtPub;
    usrtPub.yaw = (vision_data->yaw * CV_PI) / 180.;  //角度制转弧度制
    usrtPub.pitch = (vision_data->pitch * CV_PI) / 180.;
    usrtPub.enemycolor = vision_data->enemycolor;
    usrtPub.lockcommand = vision_data->lockcommand;  //没啥用
    usrtPub.nominal_bulletspeed = vision_data->nominal_bulletspeed * 100;

    usrtPub.bullet_speed =
        vision_data->bullet_speed < 26 ? 2600 : vision_data->bullet_speed * 100;
    usrtPub.beat_mode = vision_data->beat_mode;
    usrtPub.buff_type = vision_data->buff_type;
    usrtPub.colorvincible = vision_data->colorvincible;
    usrtPub.ifbeatsentry = vision_data->enemycolor;
    usrtPub.recv_header.stamp =
        tdttoolkit::Time::GetRosTimeByTime(vision_data->time_stamp);
    // std::cout << "usrtPub.yaw= " << usrtPub.yaw << std::endl;
    // std::cout << "usrtPub.pitch= " << usrtPub.pitch << std::endl;
    // std::cout << "usrtPub.bullet_speed= " << usrtPub.bullet_speed <<
    // std::endl;

    usartPub->publish(usrtPub);
  }

  rclcpp::Publisher<vision_interface::msg::UsartPub>::SharedPtr usartPub;
};
}  // namespace tdtusart

#endif
