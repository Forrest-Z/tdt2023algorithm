#ifndef __VISION_USART_SENDER_H
#define __VISION_USART_SENDER_H

#include <boost/asio.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "vision_interface/msg/predict_pub.hpp"

// sender 向串口发送消息
namespace tdtusart {
class VisionUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ = node->create_subscription<vision_interface::msg::PredictPub>(
        "predictPub",  // topic_name,
        10,            // buff_size,
        std::bind(&VisionUsartSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }
#pragma pack(1)
  struct VisionData {
    uint8_t header = 0xA5;
    uint8_t type = 2;  //第二种UsartSender

    float yaw;    // yaw角度
    float pitch;  // pitch角度
    uint8_t no_obj : 1;
    uint8_t beat : 1;  //开火指令

    double time_stamp;  //视觉时间系下时间戳，用于给电控做时间同步用
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // tdttoolkit::BaseCommunicator *communicator;
  rclcpp::Subscription<vision_interface::msg::PredictPub>::SharedPtr
      subscriber_;

  std::function<bool(const void *, int)> usartSend_;

  int frame_id = 0;

  void Callback(const vision_interface::msg::PredictPub::ConstPtr msg) {
    VisionData send_data;
    send_data.yaw = (msg->yaw * 180.) / CV_PI;  //弧度制转角度制
    send_data.pitch = (msg->pitch * 180.) / CV_PI;
    send_data.no_obj = msg->no_obj;
    send_data.beat = msg->beat;
    tdttoolkit::MatchInfo match_info;
    tdttoolkit::BaseBlackboard::Get("match_info", match_info);
    if (match_info.match_time < 0 && match_info.match_time > -90) {
      send_data.beat = 0;
    }
    send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    // TDT_INFO("Send Vision Data %f %f ", send_data.yaw, send_data.pitch);
  }
};

}  // namespace tdtusart
#endif