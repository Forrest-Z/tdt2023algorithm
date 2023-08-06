#ifndef __ROBOTS_PERCEPTION_H__
#define __ROBOTS_PERCEPTION_H__
#define use_tensorrt
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "armor_detect/armor_detect.h"
#include "perception_interface/msg/perception2_local.hpp"
#include "perception_interface/msg/perception2_nav.hpp"
#include "perception_interface/msg/perception_rect.hpp"
#include "perception_interface/msg/perception_robot.hpp"
#include "roborts_camera/camera.h"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_param.h"
#include "roborts_utils/base_toolkit.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "robots_perception/record.h"
#ifdef use_openvino
#include "robots_perception/openvino_Yolo.h"
#include "robots_perception/openvino_number_detector.h"
#endif
#ifdef use_tensorrt
#include "trt_number_detector.h"
#include "trt_yolo_detector.h"
#endif
namespace robots_perception {
// class Robot {
// public:
//     Robot() = default;
//     float                    getConf() const { return conf; }
//     void                     setConf(float value) { conf = value; }
//     void                     setRobotRect(const cv::Rect2f &value) {
//     RobotRect = value; } void                     setArmorRect(const
//     cv::Rect2f &value) { ArmorRect = value; } void setCentroid(const
//     cv::Point3f &value) { Centroid = value; } void setWorldCentroid(const
//     cv::Point3f &value) { worldCentroid = value; } void setCamCentroid(const
//     cv::Point3f &value) { camCentroid = value; } void setPixelCentroid(const
//     cv::Point2f &value) { pixelCentroid = value; } void setProCentroid(const
//     cv::Point2f &value) { proCentroid = value; } void setLivoxPro(const
//     cv::Point2f &value) { LivoxPro = value; } void setPointsCloud(const
//     std::vector<cv::Point3f> &value) { PointsCloud = value; } void
//     setCameraID(const uint8_t &value) { cameraID = value; } void
//     setRect(const cv::Rect2f &value) { rect = value; } void setArmorID(const
//     int &value) { ArmorID = value; } void                     setColor(const
//     int &value) { color = value; } cv::Rect2f               getRobotRect()
//     const { return RobotRect; } cv::Rect2f               getArmorRect() const
//     { return ArmorRect; } cv::Point3f              getCentroid() const {
//     return Centroid; } cv::Point3f              getWorldCentroid() const {
//     return worldCentroid; } cv::Point3f              getCamCentroid() const {
//     return camCentroid; } cv::Point2f              getPixelCentroid() const {
//     return pixelCentroid; } cv::Point2f              getProCentroid() const {
//     return proCentroid; } cv::Point2f              getLivoxPro() const {
//     return LivoxPro; } std::vector<cv::Point3f> getPointsCloud() const {
//     return PointsCloud; } uint8_t                  getCameraID() const {
//     return cameraID; } cv::Rect2f               getRect() const { return
//     rect; } int                      getArmorID() const { return ArmorID; }
//     int                      getColor() const { return color; }

// private:
struct Robot {
  int armorCount = 0;
  cv::Rect2f armorRect;               //装甲板矩形ROI
  cv::Rect2f robotRect;               //整辆车ROI
  bool Robot_With_Armor = false;      //有装甲板的机器人
  bool Robots_Without_Armor = false;  //没有装甲板的机器人
  bool Only_Armor = false;            //只有装甲板没有机器人
  float conf;                         //车子的置信度
  cv::Point3f livoxCentroid;          //雷达系
  cv::Point3f worldCentroid;          //世界系
  cv::Point3f camCentroid;            //相机系
  cv::Point2f pixelCentroid;          // yolo框中点
  cv::Point2f proCentroid;            //相机三维投影二维中点
  cv::Point2f livoxProject;
  std::vector<cv::Point3f> pointsCloud;  //机器人三维点云
  int cameraID;
  cv::Rect2f rect;  // car框选
  int armorID;      // armor_id
  int color;        // 0 1 2 (红 蓝 灰)
  std::vector<cv::Point3f>
      world_point_list_;  // Calc时根据RobotType获取对应的三维坐标
  std::vector<cv::Point2f> image_point_list_;  // Calc时算出
  cv::Point3f center_Robot;
  inline std::vector<cv::Point2f> GetImagePointsList() const {
    return image_point_list_;
  }
  inline std::vector<cv::Point3f> GetWorldPointsList() const {
    return world_point_list_;
  }
  // void                            SetWorldPoints(tdttoolkit::DPRobotType
  // DProbot_type, int balance[3]);
};

class RobotsPerception : public rclcpp::Node {
 public:
  RobotsPerception();
  ~RobotsPerception();
  void close();
  void InitDeepLearing();
  void InitCamera();
  void run();
  void Lacate(std::vector<Robot> &robots);
  /**
   * @brief 匹配车和装甲板
   */
  std::vector<Robot> MatchRobots(
      std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      std::vector<trt_detector::trt_yolo_detector::Object> &detected_robots,
      cv::Mat &src, int cam_id);

  void safe_rect(cv::Rect &rect, cv::Mat &src);
  std::vector<armor_detect::ArmorDetectInfo> GetArmorInfo(
      std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors);

  void EraseDuplicate(
      std::vector<armor_detect::ArmorDetectInfo> &output_armors);

  void DPArmorTransform(const armor_detect::ArmorDetectInfo &armor_info,
                        tdttoolkit::RobotArmor &armor, int enemy_color);

  void Judgement(std::vector<tdttoolkit::RobotArmor> &robot,
                 std::vector<tdttoolkit::RobotArmor> &armor);

  std::vector<tdttoolkit::RobotArmor> Info2Armor(
      std::vector<armor_detect::ArmorDetectInfo> &armors);

  std::vector<tdttoolkit::RobotArmor> GetenemyArmor(
      std::vector<tdttoolkit::RobotArmor> &output_armors_data_package,
      cv::Mat &src);

  void CalAxis(std::vector<tdttoolkit::RobotArmor> &final_armors);

  void DP_logicBalance(std::vector<tdttoolkit::RobotArmor> &final_armors);

  void Resolve(vector<RobotArmor> &armors, int &camera_id);

  cv::Point3f Cam_Livox3D(cv::Point3f &Camera_Point, cv::Mat &T_Livox_Cam);

  std::vector<Robot> Armor2Robot(std::vector<RobotArmor> &armors);

  std::vector<robots_perception::Robot> removeDuplicate(
      std::vector<robots_perception::Robot> &robots);

  void Publish(std::vector<Robot> &Robots);

  void PerceptionDebug(std::vector<Robot> &Robots);

  void ReadData();

  bool HasData();

  void ClearDate();

  bool inFrame(cv::Rect &rect, cv::Mat &src);

  // 寻找灯条
  void DP_FindLightBar(
      const cv::Mat &src,
      std::vector<armor_detect::LightBarInfo> &output_light_bars);
  // 匹配灯条为装甲板
  void DP_DoubleLightBarDetect(
      const cv::Mat &src, std::vector<armor_detect::LightBarInfo> &light_bars,
      std::vector<armor_detect::ArmorDetectInfo> &output_armors);

  void roiFindLightbar(
      vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      cv::Mat &src);
  // 透视变化提取装甲板数字区域
  void extractNumber(
      vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      cv::Mat &src);
  bool DP_GetEligibility(const armor_detect::LightBarInfo &lightbar1,
                         const armor_detect::LightBarInfo &lightbar2);

  inline bool RectSafety(cv::Rect2i &rect, cv::Mat &src) {
    if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > src.cols ||
        rect.y + rect.height > src.rows) {
      return false;
    }
    return true;
  }

 private:
  //-----------------整体----------------
  bool debug = false;
  int enemyColor = 1;
  bool record = false;
  bool roiFindPoints = false;
  builtin_interfaces::msg::Time camera_time;

  //---------------相机、雷达数据-------------
VideoRecoder cam1_recoder;
VideoRecoder cam2_recoder;
VideoRecoder cam3_recoder;
VideoRecoder cam4_recoder;
  image_transport::Subscriber img_sub_1;
  image_transport::Subscriber img_sub_2;
  image_transport::Subscriber img_sub_3;
  image_transport::Subscriber img_sub_4;
  void image_callback1(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void image_callback2(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void image_callback3(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void image_callback4(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  tdtcamera::Camera *camera1;
  tdtcamera::Camera *camera2;
  tdtcamera::Camera *camera3;
  tdtcamera::Camera *camera4;
  tdtcamera::TImage frame1;
  tdtcamera::TImage frame2;
  tdtcamera::TImage frame3;
  tdtcamera::TImage frame4;
  std::vector<cv::Mat> images;
  cv::Mat empty_frame = cv::Mat::zeros(32, 32, CV_8UC3);

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher1;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher2;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher3;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher4;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr Mappublisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_pointcloud_;
  void showTheshold(std::vector<cv::Mat> &images);
  //  std::unique_ptr<image_transport::ImageTransport> it_;
  //   rclcpp::TimerBase::SharedPtr timer_;
  //   image_transport::Publisher publisher1_;
  //   image_transport::Publisher publisher2_;
  //   image_transport::Publisher publisher3_;
  //   image_transport::Publisher publisher4_;
  //   image_transport::Publisher publisher_map_;

  // rclcpp::Publisher<image_transport::>::SharedPtr
  //     compressed_image_pub1;
  // rclcpp::Publisher<image_transport::ImageCompressed>::SharedPtr
  //     compressed_image_pub2;
  // rclcpp::Publisher<image_transport::ImageCompressed>::SharedPtr
  //     compressed_image_pub3;
  // rclcpp::Publisher<image_transport::ImageCompressed>::SharedPtr
  //     compressed_image_pub4;
  // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::mutex image_mutex1;
  std::mutex image_mutex2;
  std::mutex image_mutex3;
  std::mutex image_mutex4;
  cv::Mat image_buff1;
  cv::Mat image_buff2;
  cv::Mat image_buff3;
  cv::Mat image_buff4;
  //相机雷达部分参数
  std::vector<cv::Mat> matrix;
  std::vector<cv::Mat> distCoeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<cv::Mat> T_Livox_Cam;
  std::vector<cv::Mat> T_Cam_Livox;
  std::vector<cv::Mat> rotation_matrix_camera_to_world;
  std::vector<cv::Mat> tvec_camera_in_world;

  cv::Mat Round_Map;
  rclcpp::Publisher<perception_interface::msg::Perception2Local>::SharedPtr
      pub_Local;
  rclcpp::Publisher<perception_interface::msg::Perception2Nav>::SharedPtr
      pub_Nav;
  rclcpp::Publisher<perception_interface::msg::Perception2Nav>::SharedPtr
      pub_Vision;

  /**
   * @brief 去除有装甲板类别的重复车
   * @param &robots
   */
  //---------------DeepLearing-------------

 private:
#ifdef use_openvino
  robots_perception::YoloDetector *yolo_detector = NULL;
  robots_perception::openvino_number_detector *openvino_number_detector = NULL;
#endif
#ifdef use_tensorrt
  trt_detector::trt_yolo_detector *trt_yolo_detector = NULL;
  trt_detector::trt_number_detector *trt_number_detector = NULL;
#endif
  cv::Rect2i armor_detect_roi_;  //用于检测装甲板的ROI区域
  cv::Rect2i src_area_;          //原图范围,初始化之后不得修改
  int enemy_color_;  //敌方颜色, 按照cv颜色通道顺序,0代表蓝, 1代表绿, 2代表红
  int threshold_ = 0;
  int baitian_ = 0;
  int lock_ = 0;
  cv::Mat element_;  //滤波处理核心
  cv::Mat element2_;
  int deeplearning_mode = 0;
  int gpu_mode = 0;
  int src_width = 0;
  int src_height = 0;
  float dp_cof_thre = 0;
  float dp_nms_area_thre = 0;
  static int balance_on;
  double d_avgtime = 0.0;
  int d_count = 0;
  double n_avgtime = 0.0;
  int n_count = 0;
};

}  // namespace robots_perception
#endif  // __ROBOTS_PERCEPTION_H__