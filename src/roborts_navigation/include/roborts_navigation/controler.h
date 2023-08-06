#ifndef __CONTROLER_H__
#define __CONTROLER_H__

// #include <mutex>
#include <roborts_utils/base_toolkit.h>

#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "astar_frontend.h"
#include "direct_backend.h"
#include "eigen3/Eigen/Dense"
#include "multi_polyfit_backend.h"
#include "navigation_debug.h"
#include "navigation_interface/msg/navigation2_usart.hpp"
#include "polyfit_backend.h"
namespace navigation {
class Controler {
 public:
#if USE_HYBRID_ASTAR
  using Result = HybridAstarBackend::Result;
#else
  using Result = DirectBackend::Result;
#endif

 private:
  Result result_;
  double start_time_ = 0;

  struct RobotStatus {
    double pos_x;
    double pos_y;
    double speed_x;
    double speed_y;
    double time_stamp = 0;  // 位置信息对应的时间戳
  } robot_status_;

  struct RobotEuler {
    double yaw;
    double pitch;
    double roll;
    double timestamp = 0;
  } robot_euler_;

  tdttoolkit::TaskScheduler task_schuduler_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<navigation_interface::msg::Navigation2Usart>::SharedPtr
      usart_publisher;

 private:
  int control_fps_ = 250;

  double dis_per_pixel_ = 0.25;  // 单位cm/pixel

  double regen_time_ = 1;  // 单位s

  double max_turn_speed;

  double max_acc;

  bool on_arrive = false;

  int target_type = 'Z';

  std::mutex mutex_;

  bool stop_ = false;

  tdttoolkit::Vec2d last_speed;

  void update();

 private:
  NavigationDebug *debug_;

 public:
  Controler(double dis_per_pixel, NavigationDebug *debug);

  Controler(){};

  Controler(const Controler &other);

  /*************
   * @name set_result
   * @brief 更新后端优化后的路径
   * @param [in] result 后端优化后路径
   * @param [in] start_time 路径开始时间，单位s
   */
  void set_result(const Result &result, const double &start_time);

  /*************
   * @name set_robot_pos
   * @brief 更新机器人位置
   * @param [in] x 机器人位置
   * @param [in] y 机器人位置
   * @param [in] time_stamp 该位置对应时间戳 单位s
   */
  void set_robot_pos(const double &x, const double &y,
                     const double &time_stamp);

  void set_robot_speed(const double &x, const double &y,
                       const double &time_stamp);

  void set_robot_euler(const double &yaw, const double &pitch,
                       const double &roll, const double &time_stamp);

  void set_arrive_status(const bool &status) { on_arrive = status; };

  void set_target_type(const int &type) { target_type = type; };

  void set_stop();

  void init();

  void start();

  void stop();

  tdttoolkit::Vec2d GetOutputSpeed();
};
}  // namespace navigation

#endif