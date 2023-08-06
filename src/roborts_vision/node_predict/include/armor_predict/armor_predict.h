#ifndef VISION_ARMOR_PREDICT_H
#define VISION_ARMOR_PREDICT_H

#include "base_predict/base_predict.h"
#include "vision_interface/msg/resolve_pub.hpp"
#include "perception_interface/msg/perception2_nav.hpp"
namespace armor_predict {
/*
RoboType：
0 未知    6 前哨站
1 英雄    7 哨兵
2 工程    8 基地
3，4，5 步兵
*/

struct Armor {
  // 详细注释见ResolvedArmor.msg

  int armorType = 0;                 //装甲板数字
  int armorTag = -1;                 //装甲板tag，预测使用
  double timeStap = 0;               //时间戳,单位s
  cv::Point2f armor_point_image[6];  //灯条六个点
  cv::Point2f armor_center_point;  //装甲板中心点（由灯条中心点组成）

  cv::Mat tvec_world_armor = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat rvec_world_armor = cv::Mat::zeros(3, 1, CV_32F);

  cv::Mat tvec_camera_armor = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat rvec_camera_armor = cv::Mat::zeros(3, 1, CV_32F);

  cv::Mat armorLinerVel = cv::Mat::zeros(3, 1, CV_32F);  //装甲板x,y,z的线速度
  inline float getArea() {
    float Length = abs(armor_point_image[4].x - armor_point_image[0].x);
    float Width = abs(armor_point_image[4].y - armor_point_image[0].y);
    return Length * Width;
  }

  inline cv::Point3f getRectangular() {
    // std::cout << "type= " << armorType << std::endl;
    // std::cout << "tvec_world_armor= " << tvec_world_armor << std::endl;
    return cv::Point3f(tvec_world_armor.at<float>(0, 0),
                       tvec_world_armor.at<float>(1, 0),
                       tvec_world_armor.at<float>(2, 0));
  }

  inline float getLinerVel() {
    return sqrt(pow(armorLinerVel.at<float>(0, 0), 2) +
                pow(armorLinerVel.at<float>(2, 0), 2));
  }

  inline float GetlightCenter_dis_() {
    return fabs(armor_point_image[2].x - armor_point_image[5].x);
  }
};
struct robotsAxis {
  int robotType = 0;                               // 装甲板类型
  std::deque<std::pair<cv::Point3f, float>> axis;  // 轴心 cm、s

  std::deque<std::pair<float, float>> radiusLength[2];
  // float radiusLength[2] = {
  //     25, 25};  // 机器人轴心。单位：cm 奇偶代表两种轴长（长短不一）
  //               // 平衡只有一个轴长

  const float outPostRadius = 25.685;
  const float defaultRadius = 25.;
  inline float getRadius(int armorType, int armorTag) {
    if (armorType == 6)  // 前哨站取固定轴长
      return outPostRadius;
    else
      // return 25 ;
      return statePostRadius[armorTag % 2].first;
  }
  inline cv::Point3f getAxis(bool lastAxis = false) {
    if (lastAxis && axis.size() > 1) {
      return axis[axis.size() - 2].first;
    } else if (axis.empty()) {
      return cv::Point3f(-1, -1, -1);
    } else
      return axis.back().first;
  }

  void clear() {
    selfTurn = 0;
    statePostRadius[0] = std::make_pair(25, 0);
    statePostRadius[0] = std::make_pair(25, 0);
    axis.clear();
    radiusLength[0].clear();
    radiusLength[1].clear();
    radiusFilter.init(defaultRadius);
    axisFilter.init(cv::Point3f(-1, -1, -1));
  }
  int selfTurn = 0;  // 0->未知  1->逆时针 -1->顺时针

  tdttoolkit::EKF axisFilter;
  tdttoolkit::KalmanFilter radiusFilter;

  std::pair<cv::Mat, float> statePostAxis;
  std::pair<float, float> statePostRadius[2];
};

struct Robot {
  int roboType = 0;  // 机器人类型

  std::deque<Armor> armors[4];             // 四块tag的信息
  std::pair<cv::Mat, float> statePost[4];  // 四个装甲板后验估计  cm/s  , s
  tdttoolkit::EKF ekfFilter[4];

  std::vector<Armor>
      lastArmors;  // 上一帧识别的装甲板(对于标tag而言是上一帧，但对于预测而言是当前帧。因为标tag后更新lastArmors为当前帧装甲板)

  // Axis axis;  //轴心

  // RobotRadius robotRadius;  //轴长
  // normalInfo normalInfo_[4];  //装甲板运动信息

  /**************************************************************************
   * @name            InitializationKalmanFilter
   * @brief           初始化卡尔曼滤波器
   *
   ***************************************************************************/
  inline void InitializationKalmanFilter() {  // TODO ：这个不要为 -1，-1，-1
                                              // ，会使statePost初始化为0，
    for (int i = 0; i < 4; i++) {
      ekfFilter[i].init(cv::Point3f(-1., -1., -1.));
    }
  }

  bool changeTag = true;  //换向标志位
  cv::Point2f robotCenterInImage = cv::Point2f(0, 0);
  int lastTag = 0;  // 上次进入轴心中心的装甲板tag
  int lastChosenTag = -1;
  robotsAxis robotsAxis_;  // 机器人轴心
};

struct MyCostFunction {
  template <typename T>
  bool operator()(const T *const x, const T *const y, T *residual) const {
    cv::Point_<T> l1 = cv::Point_<T>(x[0] - T(armor1_.x), y[0] - T(armor1_.y)),
                  l2 = cv::Point_<T>(x[0] - T(armor2_.x), y[0] - T(armor2_.y));

    T cosTheta =  //两装甲板法线夹角的 cos
        (l1.x * l2.x + l1.y * l2.y) /
        (sqrt(l1.x * l1.x + l1.y * l1.y) * sqrt(l2.x * l2.x + l2.y * l2.y));
    T sinTheta =
        (k_ == FLT_MAX ? T(0)
                       : (x[0] * T(k_) - y[0]) /
                             (T(sqrt(1 + k_ * k_)) *
                              sqrt(x[0] * x[0] +
                                   y[0] * y[0])));  //轴心yaw角与车中心yaw角 sin
    // T cosTheta = l1.dot(l2)/(cv::norm(l1)*cv::norm(l2));
    // T cosTheta = T(l1.dot(l2) / (cv::norm(l1) * cv::norm(l2)));
    residual[0] = cosTheta + sinTheta;
    return true;
  }
  MyCostFunction(cv::Point2f armor1, cv::Point2f armor2, float k)
      : armor1_(armor1), armor2_(armor2), k_(k) {}

  cv::Point2f armor1_, armor2_;
  float k_ = 0;
};

class armor_predict : public base_predict::base_predict {
 public:
  /*
  @name armor_predict
  @brief 构造函数，注册订阅与发布
  */
  armor_predict(rclcpp::Node::SharedPtr &predict_node);

  void init() override;

  /*
  @name resolve_callback
  @brief 订阅解算数据
  */
  void resolve_callback(const vision_interface::msg::ResolvePub &msg);

  /*
  @brief 订阅解算信息
  */
  void armor_resolve_sub(const vision_interface::msg::ResolvePub &msg);

  /*
  @brief:感知订阅
  */
  void perception_sub(const perception_interface::msg::Perception2Nav &msg);


  /*
  @brief 订阅串口消息
  */
  void usart_sub(const vision_interface::msg::UsartPub &msg);

  /*
  @name  roborts_decision
  @brief 击打决策
  @return 击打目标 RoboTye详见 base_toolkit.h
  */
  std::pair<int, int> roborts_decision();

  /*
  @name predict
  @brief  预测
  */
  void predict() override;

  /*
  @brief 开火指令，包含击打类型更换
  */
  void firecommand();

  /**
   * @name            CalcArmorTag
   * @brief           计算装甲板tag
   * @author          郑乐2360838113@qq.com
   * */
  void CalcArmorTag(std::vector<Armor> &armors);

  /**
   * @name            CalcIou
   * @brief           计算装甲板CIou,用来判断前后帧装甲板tag的继承
   *                  Diou= Iou - (C-AUB)/C
   *                  Iou: 两矩形区域交集面积/两矩形区域并集面积
   *                  C: 两矩形区域最小外接面积
   *                  Diou ∈ [-1,1]  -1:不相交且距离很远
   * @author          郑乐2360838113@qq.com
   *
   * ZLL-TODO : 旧版识别的装甲板是正矩形 ，
   * 新版识别不提供装甲板，以灯条四点作为装甲板矩形 。 差异化还未对比
   * */
  float CalcGIou(Armor cntArmor, Armor lastArmor);

  /*
    @brief 清空所有历史信息
  */
  void clearHis_info(int roboType, int armortTag = -1);

  /*
    @brief 确定机器人击打模式
  */
  int GetStatus();

  /*
    @brief 平移击打模式
  */
  void FollowModeShoot(vision_interface::msg::PredictPub &sendMsg);

  /*
    @brief 小陀螺击打模式
  */
  void spinModeShoot(vision_interface::msg::PredictPub &msg);

  void spinModeShoot_2(vision_interface::msg::PredictPub &msg);

  //得到自旋后的点
  cv::Point3f spinPoint(cv::Point3f axis, cv::Point3f armor, float theta,
                        cv::Point3f move_axis = cv::Point3f(0, 0, 0));

  /*
    将预测点同步到云台消息
  */
  void prepareBeat(cv::Point3f predictPoint, float disHorizontal,
                   vision_interface::msg::PredictPub &msg,
                   int beatType) override;

  inline double getRosTime() {
    // 返回系统当前时间。单位:s

    auto t = predict_node_->now();
    return t.seconds();
  }

  /*
    @brief 获得后验估计
  */
  void EKF(std::vector<Armor> &armors);

  /*
    @brief 枪口偏置修正
  */
  inline void fireGunOffset(vision_interface::msg::PredictPub &msg) {
    // 恒定偏差
    msg.yaw += yawOffset;
    msg.pitch += pitchOffset;
  }

  /*
     @brief 过圈处理
     视觉恒定坐标系为 -180 ~ +180 ， 但IMU会累计角度，因此二者相差 n倍的360°
  */
  void overCircle(vision_interface::msg::PredictPub &msg);

  /*
    计算轴心
  */
  void MeasureAxis(std::vector<Armor> &armors);

  /*
    @brief 装甲板法线交于一点 计算轴心
  */
  cv::Point3f calcAxisBy_normalIntersection(Armor armor1, Armor armor2);

  /*
    @brief 装甲板法线延长计算轴心
  */
  cv::Point3f calcAxisBy_normalExtension(Armor armor);

  /*
   计算合适的中心区域装甲板
  */
  void entryPoint(std::vector<Armor> &armors);

  /*
    根据轴心和轴长反向推算装甲板
  */
  cv::Point3f DistanceCompensation(cv::Point3f axis_point, int armorType,
                                   int armorTag);

  /*
  矫正轴心，通过深度学习识别得到的车框中心像素点矫正yaw角
  */
  void correctAxis(cv::Point3f &axisInWorld, int roboType);

  cv::Point3f correctAxis_2_0(std::vector<cv::Point3f> &axisPoints,
                              const int roboType);

  /*
    @brief 物体坐标系 阵 转世界坐标系 的欧拉角
  */
  std::pair<cv::Vec3f, cv::Vec3f> Rotation2Euler(cv::Mat rotation);

  float getRobotYawInCamera(int roboType);

  cv::Point3f camera_TO_world(cv::Point3f axisPoint_corrected,bool ifUseTvec = true);

  

 private:  //以下是注册的订阅与发布
  rclcpp::Subscription<vision_interface::msg::ResolvePub>::SharedPtr
      armorResolveSub;
  
  rclcpp::Subscription<perception_interface::msg::Perception2Nav>::SharedPtr
      preceptionSub;
 private:            //以下是预测所需的变量
  Robot roborts[9];  //储存机器人信息

  // [fitst: 装甲板类型type]  [second:装甲板tag]
  std::pair<int, int> beatRobotype;  //决策装甲板

  cv::Mat cameraMatrix;  // 相机内参

  bool ifPredict = 1;
  double perceptionTime = 2;
  double continusTime = 0.5;       // 持续时间,单位: s
  double sameArmorScore = -0.367;  // 装甲板满足继承条件阈值
  int dequeMaxNum = 15;            // 规定队列最大数目为30
  float yawOffset = 0;
  float pitchOffset = 0;
  float minLinerVel = 30;  // cm/s
  int armorDecisionAttenuationCoefficient = 5;
  float armorRecvPitch = 0.2617994;  //单位：弧度 装甲板倾斜角度
  float robotLength = 35;  //单位：cm ,车的平均长度58cm,这里取2/3长度
  float fireLength = 12.;

  float  decision_yaw =0;
  float init_pitch = 0 ;
  double lastDetectTime = 0;

  // float continueTime = 0.5 ; //延迟时间 ，
  // 当在这个时间内，将会对历史信息进行预测。超过这个时间将会发送云台当前角度

  // spinParams
  float spinTimeOut = 0.5;        // 判断是否进入小陀螺时用
  float lastEnterCenterTime = 0;  // 上一次装甲板进入中心区域时间
  float spinMoveMinDelatYaw = 0.03;  //小陀螺移动的最小delta_yaw
  int spinModeByDeepLearning = 1;  // 小陀螺模式深度学习进行轴心跟随
  float calcVelthreshold = 1;      //计算转速的阈值
  float centerTagthreshold = 0.5;  //装甲板进入中心的标志，数值越小条件越苛刻
  float spinPreScale = 0.85;  //小陀螺决策击打装甲板参数，数值越小越容易切换
  float spinBeatMinAngle = 25;  //小陀螺开火最小角度
  float radiusFilter_Q = 0.45;
  float radiusFilter_R = 0.1;

  // ceres solver
  ceres::Problem problem;

  ceres::CostFunction *cost_function = NULL;

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  //precption

  std::pair <std::vector<std::pair<cv::Point3f,int > >,double >precption_robot ;
};

}  // namespace armor_predict

#endif