#ifndef VISION_BASE_DETECT_H
#define VISION_BASE_DETECT_H

#include <cv_bridge/cv_bridge.h>
#include <linux/input.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "base_interface/msg/feed_back_param.hpp"
#include "vision_interface/msg/image_pub.hpp"
#include "vision_interface/msg/predict_pub.hpp"
#include "vision_interface/msg/usart_pub.hpp"

// #include "rclcpp_action/rclcpp_action.hpp"

#include "roborts_utils/debug.h"

// #include "tdtdebug/debug.h"
// #include "toolkit/base_class.h"
// #include "toolkit/base_toolkit.h"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_toolkit.h"
namespace base_detect {

class base_detect {
 public:
  base_detect(){};

  virtual void init() = 0;  // 初始化函数

  virtual void detect(
      const vision_interface::msg::ImagePub &img_msg) = 0;  // 检测函数

  inline int getBeatMode() { return (tdttoolkit::beatMode)(recvMsg.beat_mode); }

  bool open_debug = false;  //开debug的开关

  virtual void initParam() = 0;

  vision_interface::msg::UsartPub recvMsg;  //当前帧的串口数据

  rclcpp::Node::SharedPtr detect_node_ = NULL;

  // rclcpp::Service<vision_interface::srv::RequestVisionParam>::SharedPtr service_debug = NULL;

  rclcpp::Subscription<base_interface::msg::FeedBackParam>::SharedPtr
      paramFeedBack_sub = NULL;

  // rclcpp::Publisher<vision_interface::msg::PredictPub>::SharedPtr predictPub;
};

}  // namespace base_detect

#endif