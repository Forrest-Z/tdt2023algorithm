#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "rclcpp/rclcpp.hpp"
#include "roborts_camera/tdtcamera.h"
// #include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vision_interface/msg/image_compressed.hpp"
#include "vision_interface/msg/image_pub.hpp"
#include "vision_interface/msg/usart_pub.hpp"
namespace tdtcamera {
class HikCameraNode : public rclcpp::Node {
 public:
  HikCameraNode();
  ~HikCameraNode(){};

  void PubImage();

  inline vision_interface::msg::ImagePub getImage() { return image_msg; }

  void usartCallback(const vision_interface::msg::UsartPub &msg);

  // void imencode();

 private:
  tdtcamera::HikvisionCam *camera = NULL;

  vision_interface::msg::ImagePub image_msg;

  vision_interface::msg::ImageCompressed compressed_image_msg;

  // rclcpp::Publisher<vision_interface::msg::ImagePub>::SharedPtr image_pub;
  rclcpp::Publisher<vision_interface::msg::ImageCompressed>::SharedPtr
      compressed_image_pub;

  rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr usartSub;

  vision_interface::msg::UsartPub usrtPub_;  // 串口消息
};
}  // namespace tdtcamera
#endif  // _CAMERA_H_
