#include <rclcpp/rclcpp.hpp>

#include "armor_predict/armor_predict.h"
#include "base_predict/base_predict.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  base_predict::base_predict *predict = NULL;

  rclcpp::Node::SharedPtr predict_node =
      rclcpp::Node::make_shared("VISION_PREDICT");

  armor_predict::armor_predict armor_predict(predict_node);
  // add other predict node ,such as: buff,lob .....
  predict = &armor_predict;

  while (rclcpp::ok()) {
    // 调用串口的回调函数

    rclcpp::spin_some(predict_node);
 
    switch (predict->getBeatMode()) {
      case tdttoolkit::ARMOR_MODE:  //平移
        predict = &armor_predict;
        /* code */
        break;
      case tdttoolkit::ENERGY_MODE:  //能量机关
                                     // dosomeing
        break;

      default:

        break;
    }

    // preparebeat`

    // predict->prepareBeat();

    // predict->predict() ;
  }

  rclcpp::shutdown();

  // 释放指针
  // delete predict;
  // predict = NULL;
  return 0;
}