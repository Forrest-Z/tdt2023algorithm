#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <vision_interface/msg/detail/manual_towords_control__struct.hpp>

#include "astar_frontend.h"
#include "controler.h"
#include "direct_backend.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hybrid_astar_backend.h"
#include "localization_interface/msg/lam2_nav.hpp"
#include "multi_polyfit_backend.h"
#include "navigation_debug.h"
#include "navigation_interface/msg/nav_command.hpp"
#include "navigation_interface/msg/nav_decision_command.hpp"
#include "navigation_interface/msg/navigation2_usart.hpp"
#include "navigation_interface/msg/navigation_debug.hpp"
#include "navigation_interface/msg/usart_speed2_nav.hpp"
#include "perception_interface/msg/perception2_nav.hpp"
#include "polyfit_backend.h"
#include "roborts_utils/roborts_utils.h"
#include "vision_interface/msg/manual_towords_control.hpp"

#define NAVIGATION_DEBUG

class Navigation {
 private:
  const int cache_size = 500;

  struct RobotStatus {
    double position_x = -1;
    double position_y = -1;
    double time;  // 单位 s
    double velocity_x = 0;
    double velocity_y = 0;
    double yaw = 0;
    double turn_angle = 0;

    bool pos_valid = false;
    bool vel_valid = false;

    bool operator<(const RobotStatus &other) const { return time < other.time; }
    bool operator>(const RobotStatus &other) const { return time > other.time; }
    bool operator<(const double &other) const { return time < other; }
    bool operator>(const double &other) const { return time > other; }
  } robot_status_;
  std::deque<RobotStatus> robot_status_history_;

  struct UsartSpeed {
    double speed_x = 0;
    double speed_y = 0;
    double time;  // 单位 s

    bool operator<(const UsartSpeed &other) const { return time < other.time; }
    bool operator>(const UsartSpeed &other) const { return time > other.time; }
    bool operator<(const double &other) const { return time < other; }
    bool operator>(const double &other) const { return time > other; }
  } usart_speed_;
  std::deque<UsartSpeed> usart_speed_history_;

  struct RobotCommand {
    double goal_x = -1;
    double goal_y = -1;
    int target_type = 0;

    int next_command_id = -1;
    RobotCommand(){};
    RobotCommand(double x, double y) : goal_x(x), goal_y(y){};

    RobotCommand(double x, double y, int target_type)
        : goal_x(x), goal_y(y), target_type(target_type){};

    bool operator==(const RobotCommand &other) {
      bool ret = true;
      ret = ret && goal_x == other.goal_x;
      ret = ret && goal_y == other.goal_y;
      ret = ret && target_type == other.target_type;
      ret = ret && next_command_id == other.next_command_id;
      return ret;
    }
  } robot_command_;

 private:
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<localization_interface::msg::LAM2Nav>::SharedPtr
      localization_subscriber_;
  rclcpp::Subscription<perception_interface::msg::Perception2Nav>::SharedPtr
      percep_subscriber_;
  rclcpp::Subscription<navigation_interface::msg::NavCommand>::SharedPtr
      nav_command_subscriber_;
  rclcpp::Subscription<navigation_interface::msg::NavDecisionCommand>::SharedPtr
      nav_decision_command_subscriber_;
  rclcpp::Subscription<navigation_interface::msg::UsartSpeed2Nav>::SharedPtr
      usart_speed_subscriber_;
  rclcpp::Publisher<vision_interface::msg::ManualTowordsControl>::SharedPtr
      manual_towords_control_publisher_;

  tdttoolkit::TaskScheduler task_scheduler_;
  navigation::AStar astar_;
  navigation::PolyFit polyfit_;
  navigation::MultiPolyfitBackend multipolyfit_;
  navigation::DirectBackend direct_;
  navigation::HybridAstarBackend hybrid_astar_backend_;

  cv::Mat map_;
  navigation::Controler *controler_;

  int auto_mode = 0;

#ifdef NAVIGATION_DEBUG
  navigation::NavigationDebug navigation_debug_;
  rclcpp::Subscription<navigation_interface::msg::NavigationDebug>::SharedPtr
      debug_subscriber_;
#endif

  float kExtraCost_ = 14;
  float kExtraCostHybridAstar_ = 14;
  int stepAstar_ = 5;
  int resolverRateAstar_ = 2;
  int resolverRateHybridAstar_ = 2;
  int resolverRatePolyfit_ = 5;
  int order_ = 3;
  int max_speed_ = 30;
  int max_acc_ = 0.5;  // 单位pixel/s^2

  double dis_per_pixel_ = 4;  // 单位cm/pixel

  int self_color_ = 1;  // 0->蓝方 2->红方

  float last_recv_time_ = -1;  // 单位s

  float lidar_map_start_x_ = 0;
  float lidar_map_start_y_ = 0;
  float lidar_map_start_yaw_ = 0;

  double last_update_time = 0;
  double timeTickAstar;

  int astarLeadSizeForHybrid;

  std::mutex wait_mutex_;
  std::condition_variable wait_cv_;
  std::vector<int> strategic_point_list_;
  std::vector<RobotCommand> intermediate_point_list_;

  int speicific_flag = 0;

  bool local_planner_enable = false;

  bool stop_flag = false;

#if USE_HYBRID_ASTAR
  bool hybird_astar_find_answer_ = false;
#endif

  int command_id = 0;
  std::unique_ptr<std::unique_ptr<uchar[]>[]> command_map_;
  double command_map_dis_per_pixel_ = 4;  // 单位cm/pixel

  void Update();

  void InitVelCallback(const localization_interface::msg::LAM2Nav &msg);

  void PercepCallback(const perception_interface::msg::Perception2Nav &msg);

  void NavCommandCallback(const navigation_interface::msg::NavCommand &msg);

  void NavDecisionCommandCallback(
      const navigation_interface::msg::NavDecisionCommand &msg);

  void UsartSpeedCallback(const navigation_interface::msg::UsartSpeed2Nav &msg);

  void FixLidarPos(const double &pos_x, const double &pos_y, const double &yaw,
                   double &output_pos_x, double &output_pos_y);

#ifdef NAVIGATION_DEBUG
  void DebugInitVelCallback(
      const navigation_interface::msg::NavigationDebug &msg);
#endif

 public:
  Navigation(std::shared_ptr<rclcpp::Node> ros_node);
  ~Navigation();

  void start();

  void stop();
};
#endif