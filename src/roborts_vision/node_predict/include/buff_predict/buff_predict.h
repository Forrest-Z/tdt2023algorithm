#ifndef Vision_ENERGY_PREDICT_H
#define Vision_ENERGY_PREDICT_H

#include "base_predict/base_predict.h"
#include "vision_interface/msg/resolve_pub.hpp"

namespace buff_predict {

/**
 * @class   CheckPredict_Point
 * @brief   检验预测效果所用点
 */
class CheckPredict_Point {
 public:
  CheckPredict_Point(cv::Point3f Vec, double Time) {
    this->Vec = Vec;
    this->Time = Time;
  }

  cv::Point3f Vec;  //当前预测点和R标的向量差
  double Time;      //预测点时间
};

/**
 * @class   DE_in_Buff
 * @brief   定义一个子类继承DE类
 */
class DE_in_Buff : public tdttoolkit::Differential_Evolution {
  /**
   * @brief 自定义适应度方程，对realx进行优化，使其更符合场上大符实际的运动模型
   * @note     0.785sin( 1.884t + realx[0]) + realx[1];
   * @param    realx[0]
   * @param    realx[1]官方给了这个数:1.305，但误差可能比较大
   * @return   double
   */
  double CalculateFitness(std::vector<double> realx);

 public:
  DE_in_Buff() = default;

  std::vector<double> resolve_angle_speed;  //解算出的角速度
  std::vector<double> resolve_time;         //解算时间
  bool type = 0;                            // true是大符，false是小符
};

class buff_predict : public base_predict::base_predict {
  buff_predict(rclcpp::Node::SharedPtr &predict_node);

  void init() override;

  /*
  @name resolve_callback
  @brief 订阅解算数据
  */
  void buff_resolve_callback(const vision_interface::msg::ResolvePub &msg);

  /*
  @brief 订阅串口消息
  */
  void buff_usart_sub(const vision_interface::msg::UsartPub &msg);

  /**
   * @brief    计算要输出的信息(包括重力补偿 角度结算 开火控制)
   * @param    shootplatform  云台
   * @param    message 对message成员变量进行赋值
   * @return   void
   */
  void BuffPredict(tdttoolkit::ReceiveMessage &receive_message,
                   tdtusart::Send_Struct_t &send_message, double detect_time);

  /**
   * @name    SetTarget
   * @brief   简单处理当前目标，比如切换叶片后清空等
   */
  void SetTarget(Buff &target, int resolve_num,
                 tdtusart::Send_Struct_t &sendStruct);

  /**
   * @name    OverCircleManage
   * @brief   过圈处理
   */
  void OverCircleManage(tdttoolkit::ReceiveMessage receiveMessage,
                        tdtusart::Send_Struct_t &sendStruct);

  /**
   * @name    BuffCommandBuild
   * @brief
   */
  void BuffCommandBuild(tdttoolkit::ReceiveMessage recevice_message,
                        tdtusart::Send_Struct_t &send_message);

  void BeatCommand(tdttoolkit::ReceiveMessage recevice_message,
                   tdtusart::Send_Struct_t &send_message);

  /**
   * @name    SettingDE
   * @brief   设置差分进化的参数
   */
  void SettingDE();

  /**
   * @brief   判断能量机关类型：大符和小符
   */
  void JudgeType(tdttoolkit::ReceiveMessage &receive_message);

  void CheckFuc();

  /**
   * @brief   判断能量机关方向：顺时针和逆时针
   */
  void JudgeDirection();

  /**
   * @name    PolyFit
   * @brief   能量机关的拟合，包括大小符
   */
  void PolyFit(Buff &target);

  /**
   * @name    Init
   * @brief   [第一次进入能量机关状态]和[从别的状态切入能量机关状态]时执行
   * @brief 确保重新进入能量机关模式时能重新判断类型和方向,并且重新进行函数拟合
   */
  void Init();

  std::pair<float, float> GetRYawPitch() {
    std::pair<float, float> average_yp = {0, 0};
    for (int i = 0; i < R_yaw_pitch.size(); i++) {
      average_yp.first += R_yaw_pitch[i].first;
      average_yp.second += R_yaw_pitch[i].second;
    }
    average_yp.first /= R_yaw_pitch.size();
    average_yp.second /= R_yaw_pitch.size();
    return average_yp;
  }

  float GetLastBeatTime() { return last_beat_time_; }

 private:
  /**
   * @name    Predict
   * @brief   通过        void JudgeIsFinal();
  预测时间计算大概击打位置,推算就出diff_pitch,得到settime(提前静态瞄准时间)
  */
  void Predict(tdttoolkit::ReceiveMessage &receive_message, double detect_time);

  /**
   * @name    CalcPredictPoint
   * @return  预测点在物体坐标系下的三维坐标点
   */
  cv::Point3f CalcPredictPoint(float &predict_time,
                               tdttoolkit::ReceiveMessage &receive_message);

  /**
   * @brief   将x,y方向的允许误差转化至圆的切向和径向
   */
  void DetaConvert(const double angle, const double deta_x, const double deta_y,
                   double &deta_radial, double &deta_tangent);

  /**
   * @brief   滑动窗口滤波
   */
  float ShiftWindowFilter(int start_idx);

  /**
   * @name log日志
   */
  void BuffLog(tdttoolkit::ReceiveMessage &receive_message,
               tdtusart::Send_Struct_t &send_message);

  std::vector<Buff> history_targets_;  //储存历史的BuffTarget;
  std::vector<float> buff_speeds_;     //存放能量机关角速度

  float sin_A = 0.785;
  float sin_B = 1.884;
  float sin_p = 0.1;
  float sin_o = 1.305;
  //预测点
  bool first_time = true;
  bool is_beat = false;
  int wait_mode = true;        // true为等待击打,false为跟随击打
  float beat_time = 0;         //击打时间 单位:ms
  cv::Point3f predict_point_;  //预测点
  float predict_time_ = 700;   //单位:ms
  float predict_point_angle_;  // 预测位置大符姿态，用于detax,detay转R,θ

  float last_time_ = 0;  //单位:s
  bool start = false;
  bool final = 0;  //是否最后一片扇叶
  int is_final_ = 0;
  float predict_angle_ = 0;
  float last_beat_time_ = 0;
  bool clockwise_ = true;   // true顺时针 false逆时针
  bool buff_type_ = false;  // true大符,false小符
  bool last_buff_type = false;
  double fly_time_ = 0.3;  //子弹飞行时间  单位：s
  std::vector<CheckPredict_Point> Check_Predict;  //检验预测效果
  double fire_delay_ = 90;                        //开火延迟     单位：ms
  double control_time_ = 450;  //枪口控制时间  单位：ms
  bool polyfit_over_ = false;  //判断拟合是否完成
  tdttoolkit::KalmanFilter P_Filter,
      O_Filter;  //差分进化后得到的2个参数的卡尔曼滤波
  float Kalman_Q, Kalman_R;  //卡尔曼滤波用的Q和R(过程噪声和测量噪声)
  int last_resolve_num = 0;  //上次解算所用叶片数(主要用于解算不同叶片时刷新)
  double last_switch_time = 0;  //切换叶片的瞬时时间 单位:ms
  std::vector<std::pair<float, float>>
      R_yaw_pitch;  //记录指向R时的yaw和pitch,用于18m/s弹速看不到目标时使用

  double first_t = 0;  // 开启大符模式到第一次开始拟合所需时间
  double polyfit_time;

  bool first_beat = false;
  float distance_ = 0;  // 目标点的距离

  float max_a = 3;      // 最大角加速度
  float max_v = 2.5;    // 最大角速度
  int window_size = 2;  // 滑动窗口大小

 private:  //以下是注册的订阅与发布
  rclcpp::Subscription<vision_interface::msg::ResolvePub>::SharedPtr
      armorResolveSub;

  rclcpp::Subscription<vision_interface::msg::UsartPub>::SharedPtr usartSub;

  rclcpp::Publisher<vision_interface::msg::PredictPub>::SharedPtr predictPub;

  vision_interface::msg::UsartPub recvMsg;
}

class disbuff_prdict : public buff_predict {
 public:
  DisturbPredictor() = default;

  // 初始化卡尔曼滤波器
  void Init();

  // 判断大符旋转方向
  void CalculateSpeed(Buff &target);

  // 决定打哪个扇叶
  void SetTarget(Buff &target, tdttoolkit::ShootPlatform shootPlatform);

  void Resolve(Buff &target, tdttoolkit::ShootPlatform shootPlatform);

  void BeatJudgement(tdttoolkit::ReceiveMessage recevice_message,
                     tdtusart::Send_Struct_t &send_message);
  // 反符对外接口
  void Predict(Buff &target, float detect_time,
               tdttoolkit::ReceiveMessage recevice_message,
               tdtusart::Send_Struct_t &send_message);

  void JudgeDirection();

  cv::Mat point3f_to_mat(const cv::Point3f &p) {
    cv::Mat mat(3, 1, CV_32F);
    mat.at<float>(0) = p.x;
    mat.at<float>(1) = p.y;
    mat.at<float>(2) = p.z;
    return mat;
  }
  cv::Point3f mat_to_point3f(const cv::Mat &mat) {
    cv::Point3f p;
    p.x = mat.at<float>(0);
    p.y = mat.at<float>(1);
    p.z = mat.at<float>(2);
    return p;
  }

 private:
  tdttoolkit::KalmanFilter Yaw_Filter_, Pitch_Filter_,
      Distance_Filter_;  //差分进化后得到的2个参数的卡尔曼滤波
  float Kalman_yaw_Q = 0, Kalman_yaw_R = 0, Kalman_pitch_Q = 0,
        Kalman_pitch_R = 0;  //卡尔曼滤波用的Q和R(过程噪声和测量噪声)
  std::vector<tdttoolkit::BuffArmor> target_armors_;  // 已经打亮的装甲板
  tdttoolkit::BuffArmor final_armor_;
  std::vector<tdttoolkit::BuffArmor> BuffArmor_history;
  std::vector<Buff> Buff_history;
  float yaw_speed_ = 0, pitch_speed_ = 0,
        distance_ = 0;  // 装甲板在极坐标系yaw和pitch的速度和距离

  float posterior_yaw_speed_ = 0, posterior_pitch_speed_ = 0,
        posterior_distance_ = 0;  // 卡尔曼滤波后的水平竖直速度和距离

  float fly_time_ = 0.2;
  float predict_time_ = 0.3;
  float last_beat_time_ = 0;

  vector<cv::Point3f>
      buffarmor_world_middlepoints;  //装甲板左（0）右（1）两边中点世界坐标
  vector<cv::Point2f>
      buffarmor_image_middlepoints;  ////装甲板左（0）右（1）两边中点像素坐标
  vector<float> buff_speed;
  int clockwise = 0;
  float dangle;
  vector<Mat> rvec_obj2world;
};
}  // namespace buff_predict

#endif