#ifndef __GYROSCOPE_USART_RECVER_H
#define __GYROSCOPE_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <vector>

#include "base_usart.h"
#include "roborts_utils/roborts_utils.h"

namespace tdtusart {
class GyroscopeUsartRecver : public BaseUsartRecver {
 public:
#pragma pack(1)
  struct GyroscopeData {
    uint8_t header = 0xA5;
    uint8_t type = 1;
    int16_t accOriginData_x;
    int16_t accOriginData_y;
    int16_t accOriginData_z;
    int16_t gyroOrinData_x;
    int16_t gyroOrinData_y;
    int16_t gyroOrinData_z;
    float yaw;
    float pitch;
    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(GyroscopeData); }

  void ParseData(void *message) override {
    GyroscopeData *gyroscope_data = (GyroscopeData *)message;
    // TDT_INFO("Recv Gyro Data %d %d %d", gyroscope_data->accOriginData_x,
    //          gyroscope_data->accOriginData_y,
    //          gyroscope_data->accOriginData_z);
  }

  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override{

  }
};
}  // namespace tdtusart

#endif