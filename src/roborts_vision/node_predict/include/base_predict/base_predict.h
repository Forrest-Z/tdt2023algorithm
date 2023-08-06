#ifndef VISION_BASE_PREDICT_H
#define VISION_BASE_PREDICT_H

#include <ceres/ceres.h>

#include <algorithm>
#include <deque>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_param.h"
#include "vision_interface/msg/predict_pub.hpp"
#include "vision_interface/msg/usart_pub.hpp"

// openMP 加速
#include <omp.h>

namespace base_predict {

class base_predict {
 public:
  base_predict();

  virtual void init() = 0;  // 初始化函数

  // virtual int getBeatMode() = 0;

  virtual void predict() = 0;

  virtual void prepareBeat(cv::Point3f predictPoint, float disHorizontal,
                           vision_interface::msg::PredictPub &msg,
                           int beatType) = 0;

  inline void setBeatMode(int MODE) { beatMode = MODE; }

  inline int getBeatMode() { return (tdttoolkit::beatMode)beatMode; }

  /*
     获得ROS时间 ， 单位 ：s
  */
  inline double getTimeNow() {
    // double cntTime = recvMsg.recv_header.stamp.sec +
    //                  recvMsg.recv_header.stamp.nanosec * 1e-9;
    double cntTime = predict_node_->now().seconds();
    //+
    //                 predict_node_->now().nanoseconds() * 1e-9;
    return cntTime;
  }

  rclcpp::Node::SharedPtr predict_node_ = NULL;

  int beatMode = 0;

  rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr usartSub;

  rclcpp::Publisher<vision_interface::msg::PredictPub>::SharedPtr predictPub;

  vision_interface::msg::UsartPub recvMsg;  // ZLL-TODO : 消息未进行处理 ，
                                            // 。决定取消FOV限制，但未测试

  tdttoolkit::ShootPlatform platForm;  //识别这一帧画面时云台的yaw与pitch角

  cv::Mat TvecCameraInWorld_;  //世界坐标下相机的位置
};

}  // namespace base_predict
#endif