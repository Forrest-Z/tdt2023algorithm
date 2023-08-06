#include <rclcpp/rclcpp.hpp>

#include "armor_detect/armor_detect.h"
#include "base_detect/base_detect.h"
#include "roborts_camera/camera.h"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  LoadParam::InitParam("detect", "./config/vision_param.jsonc");

  base_detect::base_detect *detect = NULL;

  rclcpp::Node::SharedPtr detect_node =
      rclcpp::Node::make_shared("VISION_DETECT_NODE");

  armor_detect::ArmorDetector armor_detect(detect_node);

  detect = &armor_detect;
  // add other predict node ,such as: buff,lob .....

  auto camera_node = std::make_shared<tdtcamera::HikCameraNode>();

  rclcpp::WallRate loop_rate(100); // 循环频率为100Hz

  // tdtcamera::HikCameraNode camera;

  while (rclcpp::ok()) {

    //现在逻辑： 获取IMU数据 -> 获得相机画面 ->判断识别类型 -> 进行识别并发布 

    rclcpp::spin_some(camera_node);

    camera_node->PubImage();

    rclcpp::spin_some(detect_node);

    switch (detect->getBeatMode()) {
      case tdttoolkit::ARMOR_MODE:
        detect = &armor_detect;
        break;

      case tdttoolkit::ENERGY_MODE:

        break;
    }
    detect->detect(camera_node->getImage());
    
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  //释放指针
  // delete detect;
  // detect = NULL;
  return 0;
}
