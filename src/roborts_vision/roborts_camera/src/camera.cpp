#include "roborts_camera/camera.h"

#include <string.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "roborts_utils/base_blackboard.h"
#include "roborts_utils/config.h"

namespace tdtcamera {
HikCameraNode::HikCameraNode() : Node("camera_node") {
  
  tdtconfig::Init();
  tdttoolkit::BaseBlackboard::Init("vision_mode", sizeof(int));

  camera = new tdtcamera::HikvisionCam("./config/camera_param.jsonc");

  // image_pub =
  // this->create_publisher<vision_interface::msg::ImagePub>("image_raw", 1);

  compressed_image_pub =
      this->create_publisher<vision_interface::msg::ImageCompressed>(
          "image_jpeg", 1);

  usartSub = this->create_subscription<vision_interface::msg::UsartPub>(
      "visionUsartPub", 1,
      std::bind(&HikCameraNode::usartCallback, this, std::placeholders::_1));
}
void HikCameraNode::PubImage() {
  camera->TakeFrame(image_msg.image);

  image_msg.yaw = usrtPub_.yaw;  //云台位姿以此时刻为准
  image_msg.pitch = usrtPub_.pitch;

  image_msg.image.header.frame_id =
      camera->get_guid();  // 每个相机唯一指定一个guid

  image_msg.image.header.stamp = this->now();

  ++image_msg.seq;

  // image_pub->publish(image_msg);

  if (tdtconfig::VIDEODEBUG) {
    std::thread([this]() {
      compressed_image_msg.image.header = image_msg.image.header;

      compressed_image_msg.seq = image_msg.seq;

      compressed_image_msg.image.format = "jpeg";
      // compressed_image_msg.image.data.size = image_msg.image.data.size() / 3;

      cv::imencode(".jpg",
                   cv::Mat(image_msg.image.height, image_msg.image.width,
                           CV_8UC3, image_msg.image.data.data()),
                   compressed_image_msg.image.data);

      compressed_image_pub->publish(compressed_image_msg);
    }).detach();
  }
}

void HikCameraNode::usartCallback(const vision_interface::msg::UsartPub &msg) {
  usrtPub_ = msg;
}

}  // namespace tdtcamera
