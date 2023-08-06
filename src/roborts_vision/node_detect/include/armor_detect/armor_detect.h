#ifndef VISION_ARMOR_DETECT_H
#define VISION_ARMOR_DETECT_H
// #define use_tensorrt

#include "armor_detect/ArmorDetectInfo.h"
#include "armor_detect/LightBarInfo.h"
#include "base_detect/base_detect.h"
#include "vision_interface/msg/detect_pub.hpp"
// #include "vision_interface/msg/detect_debug.h"
#include "vision_interface/msg/detect_pub.h"
#include "vision_interface/msg/detect_pub.hpp"
#include "vision_interface/msg/image_pub.hpp"
#include "vision_interface/msg/usart_pub.hpp"
// #include "vision_interface/msg/detect_debug.h"
#include <opencv2/imgcodecs.hpp>

#include "vision_interface/msg/detect_pub.h"
#include "vision_interface/msg/image_compressed.hpp"
#include "vision_interface/msg/image_pub.hpp"
#include "vision_interface/msg/usart_pub.hpp"

// #include "tdt_config/config.h"
// #include "tdt_log/log.h"

#include "roborts_utils/config.h"
// #include "roborts_utils/config.h"

#ifdef use_openvino
#include "Yolov5faceDetector.h"
#include "number_detector.h"
#include "openvino_number_detector.h"
#endif
#ifdef use_tensorrt
#include "trt_number_detector.h"
#include "trt_yolo_detector.h"
#endif

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace tdttoolkit;

namespace armor_detect {

class ArmorDetector : public base_detect::base_detect {
 private:  // 数据类型声明
 public:
  /**
   * 完成整个检测器的初始化工作
   * @param camera
   */

  ArmorDetector(rclcpp::Node::SharedPtr &detect_node);

  /**
   * @brief
   * 完成整个检测器的初始化工作,因为在全局里面初始化时还没有读取到参数,所以在这里初始化
   *
   */
  void init() override;

  /***
   *  @brief     装甲识别调用函数
   *  @input    src  相机输入图像
   *  @input    ReceiveMessage  下位机输入信息
   *  @output   Armor    通过装甲识别的装甲板（如果没有则返回一个空装甲板）
   *自找同一类型
   *  @author   李思祁
   ***/
  std::vector<tdttoolkit::RobotArmor> Get(
      cv::Mat &src);  //之后要加入下位机的输入

  inline void Setenemy_color_(int color) { enemy_color_ = color; }
  inline int Getenemy_color_() { return enemy_color_; }

 public:
  cv::Mat traget_image[9];
  tdttoolkit::CustomRect traget_rect[9];
  cv::Mat des[9];
  int id[9];
  std::vector<LightBarInfo> trace_light_bars_;  //用于提高跟踪效果

  bool inFrame(cv::Rect &rect);

  void safe_rect(cv::Rect &rect);

 private:                //初始化后不变的量
  cv::Rect2i src_area_;  //原图范围,初始化之后不得修改
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
  double g_avgtime = 0.0;
  int g_count = 0;

 private:  //平衡车检测
  static int balance_[3];
  static int balance_counter[3][2];

 private:  //每次运行结束更新的量
  std::vector<ArmorDetectInfo>
      last_armors_info_;  //上一帧识别到的装甲板中离图像中心最近的那个
  tdttoolkit::RobotType last_robot_type_ = tdttoolkit::RobotType::
      TYPEUNKNOW;  //上一次（区别于上一帧）识别出来的机器人类型

 private:                           //每次运行前更新的量
  cv::Rect2i armor_detect_roi_;     //用于检测装甲板的ROI区域
  bool is_locking_status_ = false;  //用于记录是否锁

 private:
#ifdef use_openvino

  tdtml::YoloDetector *yolo_detector = NULL;
  tdtml::openvino_number_detector *openvino_number_detector = NULL;
  tdtml::NumberDetector *numberDetector = NULL;
#endif
#ifdef use_tensorrt
  trt_detector::trt_yolo_detector *trt_yolo_detector = NULL;
  trt_detector::trt_number_detector *trt_number_detector = NULL;
#endif

  void FindLightBar_track(const cv::Mat &src,
                          std::vector<LightBarInfo> &output_light_bars,
                          const cv::Point2i &point);

  /**
   * @name 数字识别
   * @brief 识别装甲板上的数字, 设给装甲板
   * @param src 原图
   * @param armors 装甲板们
   */
  void NumPredict(const cv::Mat &src, std::vector<ArmorDetectInfo> &armors);

  /**
   * @name 查找灯条
   * @brief 在src图像内查找灯条, 输出找到的灯条数组, 并按照从左到右排序
   * @param roi 图像
   * @param output_light_bars 灯条数组
   */
  void FindLightBar(const cv::Mat &src,
                    std::vector<LightBarInfo> &output_light_bars);

  /**
   * @name 单灯条检测
   * @brief 功能根据单个灯条去查找是非有疑似装甲板的区域,
   * 输入的灯条以从左到右排列，但是条件比较宽松适合追击哨兵的时候使用
   * @param src 原图像
   * @param light_bars 灯条, 需要以从左到右排列
   * @param output_armors 输出疑似装甲板的区域的数据包
   */
  void SingleLightBarDetect(const cv::Mat &src,
                            std::vector<LightBarInfo> &light_bars,
                            std::vector<ArmorDetectInfo> &output_armors);
  /**
   * @name 严格的单灯条检测
   * @brief 一种条件更加严格的单灯条模式，适合日常打击
   * @param src 原图像
   * @param light_bars 灯条, 需要以从左到右排列
   * @param output_armors 输出疑似装甲板的区域的数据包
   */
  void SingleLightBarDetect_tough(const cv::Mat &src,
                                  std::vector<LightBarInfo> &light_bars,
                                  std::vector<ArmorDetectInfo> &output_armors);

  /**
   * @name 双灯条查找
   * @brief 功能根据两个灯条去查找是非有疑似装甲板的区域,
   * 输入的灯条以从左到右排列
   * @param src 原图像
   * @param light_bars 灯条, 需要以从左到右排列
   * @param output_armors 输出疑似装甲板的区域的数据包
   */
  void DoubleLightBarDetect(const cv::Mat &src,
                            std::vector<LightBarInfo> &light_bars,
                            std::vector<ArmorDetectInfo> &output_armors);

  ////////////////////////////// 无情的工具函数 //////////////////////////////
#ifdef use_openvino
  std::vector<armor_detect::ArmorDetectInfo> GetArmorInfo(
      std::vector<tdtml::YoloDetector::Object> &detected_armors);

  void roiFindLightbar(vector<tdtml::YoloDetector::Object> &detected_armors,
                       cv::Mat &src);
  // 透视变化提取装甲板数字区域
  void extractNumber(vector<tdtml::YoloDetector::Object> &detected_armors,
                     cv::Mat &src);

#endif

#ifdef use_tensorrt
  std::vector<armor_detect::ArmorDetectInfo> GetArmorInfo(
      std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors);

  void roiFindLightbar(
      vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      cv::Mat &src);
  // 透视变化提取装甲板数字区域
  void extractNumber(
      vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
      cv::Mat &src);
#endif

  std::vector<tdttoolkit::RobotArmor> GetenemyArmor(
      std::vector<tdttoolkit::RobotArmor> &output_armors_data_package,
      cv::Mat &src);

  /**
   * @name 去处重复区域
   * @brief 防止由于光线干扰、画面撕裂等情况造成一个装甲板识别两次或误识别
   * @param output_armors 输出疑似装甲板的区域的数据包
   */
  void EraseDuplicate(std::vector<ArmorDetectInfo> &output_armors);

  /**
   * @brief 保证rect不出src边界
   * @note 需要this -> src_area_
   * @param rect 待判断的矩形, 如果有一部分出界,则返回其在src范围内的部分
   * @return 如果rect与src范围没有交际,返回false,否则返回true
   */
  inline bool RectSafety(cv::Rect2i &rect);

  /**
   * @brief rect中心点不变, 大小扩大到原来的gain倍(width*width, height*height),
   * 且不超出src范围
   * @note 调用RectSafety
   * @param rect 待放大的矩形
   * @param gain 放大倍数
   */
  inline cv::Rect2i RectEnlarge(const cv::Rect2i &rect, const cv::Size2i &gain);
  /**
   * @name 大津法估算阈值
   * @param input_image 输入图像
   * @param output_image 输出处理后的图像
   * @param lr 在灯条左边还是右边,
   * 用来确定数字的大概位置(-1表示该区域在灯条左边, 1表示该区域在灯条右边,
   * 0表示在中间)
   * @return 返回大津法的阈值
   */
  static int RegionOustThreshold(const cv::Mat &input_image,
                                 cv::Mat &output_image, int lr);

  /**
   * @name 确定机器学习区域
   */
  static void CalaArmorRotRect(
      ArmorDetectInfo &armor,
      tdttoolkit::CustomRect lighrbarroi = tdttoolkit::CustomRect());

  /**
   * @name 设置机器学习的ROI区域
   * @brief 为装甲板设置机器学习ROI区域
   * @param armor 装甲板
   * @param src 原图像
   */
  void GetMlRoi(const ArmorDetectInfo &armor, const cv::Mat &src,
                cv::Mat &ml_roi);

  /**
   * @name 装甲板接近?
   * @brief 判断两个装甲板是否足够靠近
   * @param armor_a a装甲板
   * @param armor_b b装甲板
   * @return 接近返回
   */
  static bool IsArmorNearby(const ArmorDetectInfo &armor_a,
                            const ArmorDetectInfo &armor_b);

  static void ArmorTransform(const ArmorDetectInfo &armor_info,
                             tdttoolkit::RobotArmor &armor);

  static void DPArmorTransform(const ArmorDetectInfo &armor_info,
                               tdttoolkit::RobotArmor &armor, int enemy_color);

  float FastTargetLock(cv::Mat target_totest,
                       tdttoolkit::CustomRect traget_rect, int test_id);

  bool GetEligibility(const LightBarInfo &lightbar1,
                      const LightBarInfo &lightbar2);

  bool DP_GetEligibility(const LightBarInfo &lightbar1,
                         const LightBarInfo &lightbar2);
  /*
   * 对预测结果进行逻辑修复
   */
  void Logic_fix(std::vector<int> &flags, std::vector<float> &confs);

  void Judgement(std::vector<tdttoolkit::RobotArmor> &armor,
                 std::vector<tdttoolkit::RobotArmor> &robot);
                 
  std::vector<tdttoolkit::RobotArmor> Info2Armor(
      std::vector<ArmorDetectInfo> &armors);

  // 当没有车框且有两个装甲板时，用装甲板计算轴心
  void CalAxis(std::vector<tdttoolkit::RobotArmor> &final_armors);
  // 在深度学习装甲板框内检测灯条点
  // void roiFindLightbar(vector<tdtml::YoloDetector::Object> &detected_armors,
  //                      cv::Mat &src);
  // // 透视变化提取装甲板数字区域
  // void extractNumber(vector<tdtml::YoloDetector::Object> &detected_armors,
  //                    cv::Mat &src);

  // 寻找灯条
  void DP_FindLightBar(const cv::Mat &src,
                       std::vector<LightBarInfo> &output_light_bars);
  // 匹配灯条为装甲板
  void DP_DoubleLightBarDetect(const cv::Mat &src,
                               std::vector<LightBarInfo> &light_bars,
                               std::vector<ArmorDetectInfo> &output_armors);

 public:
  static int BalanceSort(vector<Point2f> armors, tdttoolkit::RobotType type);
  static void logicBalance(vector<Point2f> armors, tdttoolkit::RobotType type);
  static void DP_BalanceSort(tdttoolkit::RobotArmor &armor);
  static void DP_logicBalance(vector<tdttoolkit::RobotArmor> &armors);
  static void SetBalance(int type, int balance_is) {
    if ((type > 2 && type < 6) && (balance_is == 1 || balance_is == 2)) {
      balance_[type - 3] = balance_is;
    }
  };
  static int *ReturnBalance() { return balance_; };

 private:
  rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;

  std::vector<tdttoolkit::RobotArmor> Armors;

 private
     :  // ros
        // void imageCallback(const vision_interface::msg::ImagePub &img_msg);
 private
     :  // ros
        // void imageCallback(const vision_interface::msg::ImagePub &img_msg);
  // void openDebug(const
  // std::shared_ptr<vision_interface::srv::RequestVisionParam::Request>
  // request,
  //                std::shared_ptr<vision_interface::srv::RequestVisionParam::Response>
  //                response) {
  //   //此处为动态调试参数的列表获取

  // }

  void usartCallback(const vision_interface::msg::UsartPub &usart_msg);

  void paramFeedBackCallback(
      const base_interface::msg::FeedBackParam &param_msg);
  //订阅参数反馈，通过标志位选择 更新/更新+保存参数

  void detect(const vision_interface::msg::ImagePub &img_msg) override;

  void initParam() override;

  rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr
      usart_Pub;  //订阅相机Image

  // vision_interface::msg::DetectDebug detectDebug;

  rclcpp::Publisher<vision_interface::msg::GroupMsgs>::SharedPtr qtDebug;

  rclcpp::Publisher<vision_interface::msg::DetectPub>::SharedPtr
      armor_data_pub_;  //装甲板数据发布

  rclcpp::Publisher<vision_interface::msg::ImageCompressed>::SharedPtr
      compressed_image_pub;

  //识别发送解算的数据转换函数
  void publish2resolve(uint32_t seq, float platForm_yaw, float platForm_pitch);

  std::string param_name = "detect";
};

}  // namespace armor_detect

#endif