#ifndef TDT_RESOLVE_H
#define TDT_RESOLVE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// #include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/msg/point32.hpp"
#include "vision_interface/msg/detect_pub.hpp"
#include "vision_interface/msg/resolve_pub.hpp"
#include "vision_interface/msg/resolved_armor.hpp"
#include "vision_interface/msg/usart_pub.hpp"

// #include "parameter/load_param.h"
// #include "toolkit/base_toolkit.h"
#include "roborts_utils/base_param.h"
#include "roborts_utils/base_toolkit.h"
namespace Resolve {

struct robotArmors {
  std::vector<cv::Point2f> armor_point_image;     //灯条四点像素坐标
  std::vector<cv::Point3f> armor_point_realword;  //灯条四点世界坐标
  cv::Point2f robot_center_image;                 //机器人像素中心点
  cv::Point2f armor_center_image;                 //装甲板像素中心点
  bool solvePnP_flag = false;                     // pnp解算标志位
  int armortype = 0;                              //装甲板编号
};

class ArmorResolve : public rclcpp::Node {
 public:
  ArmorResolve();

 public:
  /*
      @brief 发布解算数据
      发布数据格式： 装甲板标号
      世界坐标系下 ，物体坐标系 平移向量与旋转向量
  */
  void publish2Predict();

  /*
      @brief 订阅识别数据
  */
  void subFromDetect(const vision_interface::msg::DetectPub &msg);

  /*
  @briwef 解算正式代码
  */
  void resolve();

  /**
   * @name    UnifyCoordinate
   * @brief   将坐标描述以及角度描述统一到世界坐标系下
   * @param   pnp_rvec PNP解算获得的旋转向量
   * @param   pnp_tvec PNP解算获得的平移向量
   * @param   shootPlatform 云台当前状态
   * @param   TVecObjInWorld 本函数得出的平移向量
   * @param   RVecObjToWorld 本函数得出的旋转向量
   * @param   euler_world_to_object 世界系->物体系  Z-X-Y欧拉角
   * @param   polar_in_world 世界系->物体系  Z-X-Y欧拉角
   *
   * @齐次变换           P(物体系原点|世界系) = R(相机系 -> 世界系) *
   * P(物体系原点|相机系) + P(相机系原点|世界系)
   * @旋转矩阵的转换关系   R(物体系 -> 相机系) * R(相机系 ->　世界系) = R(物体系
   * -> 世界系)
   * @注释              P(p|B): B为上角标, 表示点p在坐标系B中的坐标
   *                   R(A -> B): A为下角标, B为上角标,
   * 表示从坐标系A的姿态旋转至坐标系B的姿态的旋转矩阵
   */
  void UnifyCoordinate(cv ::Mat &pnp_rvec, cv ::Mat &pnp_tvec,

                       cv::Mat TVecObjInWorld, cv::Mat RVecObjToWorld);

  /*
  @brief 3d->2d投影
  */
  cv::Point2f projection3d_2d();

  /*
  @brief 得到相机FOV角
  */
  std::pair<float, float> getFov();

  // void visionUsartCallBack(const vision_interface::msg::UsartPub &msg) {
  //   // usrtPub = msg;
  //   // shootPlatform.yaw = usrtPub.yaw;
  //   // shootPlatform.pitch = usrtPub.pitch;
  //   // shootPlatform.yaw = msg.yaw;
  //   // shootPlatform.pitch = msg.pitch;
  // }

 private:
  //订阅识别数据
  rclcpp::Subscription<vision_interface::msg::DetectPub>::SharedPtr
      armorResolveSub;

  // rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr usartSub;

  //进行解算，发布解算数据
  rclcpp::Publisher<vision_interface::msg::ResolvePub>::SharedPtr
      armorResolvePub;

  std::vector<robotArmors> armors;

  // vision_interface::msg::UsartPub usrtPub; // 串口消息

  vision_interface::msg::DetectPub detectPub;

  tdttoolkit::ShootPlatform shootPlatform;

  int armorsNum = 0;     //解算装甲板数目
  double armorTime = 0;  //解算时间

  //下面是解算内部数据

  // ZLL-TODO 相机在世界坐标系下的平移向量应该在LoadParam中读取,但还没做
  cv::Mat cameraMatrix;        //相机内参 size=(3,3)
  cv::Mat distCoeffs;          //相机畸变系数 size=(1,5)
  cv::Mat TvecCameraInWorld_;  //相机在世界坐标系下的平移向量 size=(3,1)
  tdttoolkit ::Polar3f polar_in_camera_;  //物体系原点在相机系中的极坐标
};

}  // namespace Resolve

#endif