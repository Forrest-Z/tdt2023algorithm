#ifndef TDT_RESOLVE_BUFF_H
#define TDT_RESOLVE_BUFF_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// #include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/msg/point32.hpp"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_param.h"
#include "roborts_utils/base_toolkit.h"
#include "vision_interface/msg/detect_pub.hpp"
#include "vision_interface/msg/resolve_pub.hpp"
#include "vision_interface/msg/resolved_armor.hpp"
#include "vision_interface/msg/usart_pub.hpp"
namespace tdtbuff {
class Buff {
 public:
  Buff();

  Buff(std::vector<tdttoolkit::BuffArmor> const &buff_armors,
       cv::Mat const &pnp_tvec_obj2cam, cv::Mat const &pnp_rvec_obj2cam,
       tdttoolkit::ShootPlatform shoot_platform, float idea_R_distance,
       float idea_yaw, bool trust_pnp_eular, bool disturb);

  void UnifyCoordinate(cv::Mat &rvec, cv::Mat &tvec,
                       tdttoolkit::ShootPlatform shootPlatform, bool to_world);

  /**
   * @brief            像素长度转角度
   * @param pixel      像素长度
   * @param axis       x 或 y 轴
   * @param cam_cv::Matrix 相机内参
   * @return           对应相机偏移角度
   */
  double Pixel2Angle(double pixel, char axis, cv::Mat cam_matrix);

  /**
   *  @brief 物理模型：通过高度差解算当前距离
   *  @param H
   * 现实实际高度差，补充：H=目标距水平地面高度差-车底盘距水平地面高度差-车轴心距车底盘高度差
   *  @param center_point  识别目标中心点像素坐标
   *  @param src_hight  相机图像大小
   *  @param cam_matrix  相机内参
   *  @param pixel_y pixel_z  相机光心到云台轴心的y，z轴方向偏置
   *  @param shoot_platform_   云台结构体
   *  @return distance 返回R标与相机光心的距离
   */
  double GetRDistance(float H, cv::Point2f center_point, int src_hight,
                      cv::Mat cam_matrix_, float pixel_y, float pixel_z,
                      tdttoolkit::ShootPlatform shoot_platform_);

  bool disturbbuff = false;

  inline std::vector<tdttoolkit::BuffArmor> GetBuffArmors() {
    return buff_armors_;
  }

  inline tdttoolkit::Polar3f GetPolar3f() { return target_world_polar; }

  inline int GetFinal() { return final_index; }

  inline bool GetEmpty() { return empty_; }

  inline float GetAngle() {
    return cv::Vec3f(this->idea_rvec_obj2world_)[2];
  };  // 真实的旋转角

  inline float GetTime() { return this->buff_armors_[0].GetTimeNow(); }

  inline float GetIdeaYaw() { return this->idea_yaw_; }

  // 反符最终要击打装甲板的极坐标
  inline void SetPolarWorld(tdttoolkit::Polar3f polar_in_world_) {
    this->polar_in_world_ = polar_in_world_;
  }

  inline tdttoolkit::Polar3f GetPolarWorld() { return this->polar_in_world_; }

  /**
   * @brief   传入当时预测点和当时R标的向量差获得预测点此刻在图像上的位置
   **/
  cv::Point GetProjectPoint(cv::Point3f Diff) {
    std::vector<cv::Point3f> obj_points;
    std::vector<cv::Point2f> pro_image_points;
    pro_image_points.clear();
    cv::Point3f objp = {0, 0, 0};
    obj_points.push_back(objp);
    cv::Point3f pre_p = GetR_IN_World() + Diff;
    cv::Point3f tmp_p = {0, 0, 0};
    cv::Mat tmp_rvec = cv::Mat(tmp_p, CV_32F);
    cv::Mat tvec_obj2_cam = cv::Mat(pre_p);
    UnifyCoordinate(tmp_rvec, tvec_obj2_cam, this->shootPlatform_, false);
    pro_image_points.clear();
    projectPoints(obj_points, tmp_rvec, tvec_obj2_cam, camera_matrix,
                  distortion_matrix, pro_image_points);
    return pro_image_points[0];
  }

  /**
   * @brief   预测旋转一定角度之后的位置
   * @param   pre_angle  预测的角度
   * @param   index      需要预测的装甲板，默认为流水灯。
   * @return  Point3f    解算旋转pre_angle之后的位置
   **/

  cv::Point3f PreAngle2WorldP(float pre_angle);

  cv::Point3f GetR_IN_World() {
    cv::Mat rvec = this->idea_rvec_obj2cam_.clone();
    cv::Mat tvec = this->idea_tvec_obj2cam_.clone();
    UnifyCoordinate(rvec, tvec, this->shootPlatform_, true);
    return tdttoolkit::MatToPoint3f(tvec);
  };

  cv::Point2f PreAngleINpixel(float pre_angle, int index = 0);

  /**
   * @brief 反符预测点在像素系的坐标
   * @param 预测点的极坐标
   */
  cv::Point2f PrePointINpixel(cv::Point3f predict_point);

  /**
   * @brief   最小化重投影误差
   * @param   PointInCamera  装甲板在相机系下的坐标
   * @param   rvec,tvec      PnP解算的物体系到相机系的旋转向量和平移向量
   **/
  void BundleAdjustment(std::vector<cv::Point3f> PointsInObject,
                        std::vector<cv::Point2f> PointsInPixel);
  void LM(cv::Mat &rtvec, const cv::Mat &J, const cv::Mat &e,
          double lambda = 1.0);

  cv::Mat camera_matrix;
  cv::Mat distortion_matrix;
  // void calc_idea_rt(bool trust_pnp_eular);

 private:
  int buff_armor_flow_;
  tdttoolkit::Polar3f target_world_polar;
  std::vector<tdttoolkit::BuffArmor> buff_armors_;

  //物体坐标系对世界坐标系(pnp)
  cv::Mat pnp_tvec_obj2world_;
  cv::Mat pnp_rvec_obj2world_;

  // pnp解算出的平移旋转矩阵
  cv::Mat pnp_tvec_obj2cam_;
  cv::Mat pnp_rvec_obj2cam_;

  //理想的平移旋转矩阵
  cv::Mat idea_tvec_obj2world_;
  cv::Mat idea_rvec_obj2world_;

  cv::Mat idea_tvec_obj2cam_;
  cv::Mat idea_rvec_obj2cam_;

  float idea_R_distance_;  //相机坐标系理想的距离
  float idea_yaw_;         //理想的yaw
  tdttoolkit::Polar3f polar_in_world_;

  bool empty_ = true;
  int final_index = 0;

  tdttoolkit::ShootPlatform shootPlatform_;
  tdttoolkit::ShootPlatform mock_shootPlatform_;

  void calc_idea_rt(bool trust_pnp_eular);

  bool VIDEODEBUG = false;
};

class BuffResolver {
 public:
  /**
   * @brief    构造函数
   **/
  explicit BuffResolver();

  /**
   * @brief    主函数中初始化用函数
   **/
  void Init();

  /**
   * @brief   大神符目标解算函数对外接口
   * @param   buff_armor
   *buff_armor,size为5,第0个是流水的,后边依次是其他的四个大神符装甲板,按顺时针顺序
   *          用多个装甲板去pnp结算可以大大提高结果精准度
   * @return  BuffTarget  输出BuffTarget类的目标
   **/
  Buff BuffResolve(std::vector<tdttoolkit::BuffArmor> &buff_armors);

  /**
   * @brief  反符目标解算函数对外接口
   * @param buff_armors 全部为无流水灯的，按像素系y坐标降序排序
   * @param receive_message
   * @return
   */
  Buff DisturbResolve(std::vector<tdttoolkit::BuffArmor> &buff_armors);

  /*
   * @brief 发布解算数据
   * 发布数据格式： 装甲板标号
   * 世界坐标系下 ，物体坐标系 平移向量与旋转向量
   */
  void publish2Predict();

  /*
      @brief 订阅识别数据
  */
  void subFromDetect(const vision_interface::msg::DetectPub &msg);

 private:
  /**
   * @brief  通过反映射纠正距离，得到世界相对物体的平移和旋转矩阵
   * @param  &buff_armor  装甲板
   * @param  &receive_message 当前云台位姿
   **/
  cv::Mat camera_matrix_;
  cv::Mat distortion_matrix_;
  int position = 0;  // 0，1，2 激活点，地面，环形高地(现在还没做)
  std::vector<float> pitch_vector;
  bool disturb = false;
  tdttoolkit::ShootPlatform shootPlatform;

  rclcpp::Subscription<vision_interface::msg::DetectPub>::SharedPtr
      buffResolveSub;
  // rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr usartSub;
  // 进行解算，发布解算数据
  rclcpp::Publisher<vision_interface::msg::ResolvePub>::SharedPtr
      buffResolvePub;
};

}  // namespace tdtbuff
#endif