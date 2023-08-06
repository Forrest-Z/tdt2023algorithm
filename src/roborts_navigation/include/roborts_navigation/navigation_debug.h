#ifndef __NAVIGATION_DEBUG_H__
#define __NAVIGATION_DEBUG_H__
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "astar_frontend.h"
#include "direct_backend.h"
#include "hybrid_astar_backend.h"
#include "multi_polyfit_backend.h"
#include "navigation_interface/msg/debug2_nav.hpp"
#include "navigation_interface/msg/navigation2_debug.hpp"
#include "navigation_interface/msg/navigation_debug.hpp"
#include "navigation_interface/srv/nav2_debug_map.hpp"
#include "polyfit_backend.h"
#include "roborts_utils/roborts_utils.h"

namespace navigation {
class NavigationDebug {
 public:
  using FrontNode = AStar::Node;
#if USE_HYBRID_ASTAR
  using Result = HybridAstarBackend::Result;
#else
  using Result = DirectBackend::Result;
#endif
  using RobotInfo = AStar::RobotInfo;

 private:
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<navigation_interface::msg::NavigationDebug>::SharedPtr
      debug_publisher_;
  rclcpp::Publisher<navigation_interface::msg::Navigation2Debug>::SharedPtr
      qtdebug_publisher_;
  rclcpp::Subscription<navigation_interface::msg::Debug2Nav>::SharedPtr
      debug2nav_subscriber_;
  rclcpp::Service<navigation_interface::srv::Nav2DebugMap>::SharedPtr
      debug_map_service;

  std::map<std::string, int*> params_;

  cv::Mat show_map_;
  size_t map_hash_;

  std::vector<FrontNode> frontnodes;
  static const int army_type_num = 8;
  Result backendresult;
  double front_update_time = 0, backend_update_time = 0;

  RobotInfo robot_info[army_type_num * 2];
  int robot_info_time_out = 0;

  float debug_scale_rate_ = 3;

  bool running_ = true;

  int radius;
  int enemy_radius;

  tdttoolkit::Vec2f position = tdttoolkit::Vec2f(-1, -1);
  float yaw = 0;
  tdttoolkit::Vec2f target = tdttoolkit::Vec2f(-1, -1);

  double dis_per_pixel_;

 private:
  void workthread();

 private:
  void OnReqNavMap(
      const std::shared_ptr<navigation_interface::srv::Nav2DebugMap::Request>
          req,
      std::shared_ptr<navigation_interface::srv::Nav2DebugMap::Response> resp);

  void OnDebug2Nav(const navigation_interface::msg::Debug2Nav::SharedPtr msg);

 public:
  NavigationDebug();

  void init();

  void add_param(const std::string& name, int& value, const int max_value = 50);

  static void OnMouse(int event, int x, int y, int flags, void* param);

  static void OnTrackbar(int value, void* param);

  void set_frontnodes(const std::vector<FrontNode>& nodes, double time) {
    frontnodes = nodes;
    front_update_time = time;
  }

  void set_backendresult(const Result& result, double time) {
    backendresult = result;
    backend_update_time = time;
  }

  void start();

  void stop();

  void update_position(const tdttoolkit::Vec2f& position, float yaw) {
    this->position = position;
    this->yaw = yaw;
  }

  void update_target(const tdttoolkit::Vec2f& target) { this->target = target; }

  void set_enemy_pos(RobotInfo enemy_pos, int enemy_id) {
    this->robot_info[enemy_id] = enemy_pos;
  }

  void publish_frame(double tick_time);
};
}  // namespace navigation

#endif