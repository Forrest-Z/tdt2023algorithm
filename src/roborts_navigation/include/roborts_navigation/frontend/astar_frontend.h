#ifndef __ASTAR_FROENT_H__
#define __ASTAR_FROENT_H__
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include "roborts_utils/roborts_utils.h"
typedef unsigned char uchar;

#define USE_HYBRID_ASTAR 1

namespace navigation {
class AStar {
 public:
  struct Node {
    float x;
    float y;
    float g;
    float h;
    float extra_cost;
    int parent_id;
    float speed_x, speed_y;
    float last_turn_angle;
    int speicific_tag;
    int x_() { return (int)x; }
    int y_() { return (int)y; }
    Node(float x, float y, float speed_x, float speed_y, float last_turn_angle,
         float g, float h, float extra_cost, int parent_id)
        : x(x),
          y(y),
          speed_x(speed_x),
          speed_y(speed_y),
          last_turn_angle(last_turn_angle),
          g(g),
          h(h),
          extra_cost(extra_cost),
          parent_id(parent_id) {}
    Node(Node old, float g, float h, float extra_cost, int parent_id)
        : x(old.x),
          y(old.y),
          speed_x(old.speed_x),
          speed_y(old.speed_y),
          last_turn_angle(old.last_turn_angle),
          g(g),
          h(h),
          extra_cost(extra_cost),
          parent_id(parent_id) {}
    Node(std::pair<float, float> pos)
        : x(pos.first),
          y(pos.second),
          speed_x(0),
          speed_y(0),
          last_turn_angle(0),
          g(0),
          h(0),
          extra_cost(0),
          parent_id(-1) {}
    Node(float x, float y)
        : x(x),
          y(y),
          speed_x(0),
          speed_y(0),
          last_turn_angle(0),
          g(0),
          h(0),
          extra_cost(0),
          parent_id(-1) {}
    Node(float x, float y, float speed_x, float speed_y, float last_turn_angle)
        : x(x),
          y(y),
          speed_x(speed_x),
          speed_y(speed_y),
          last_turn_angle(last_turn_angle),
          g(0),
          h(0),
          extra_cost(0),
          parent_id(-1) {}
    Node()
        : x(-1),
          y(-1),
          speed_x(0),
          speed_y(0),
          last_turn_angle(0),
          g(0),
          h(0),
          extra_cost(0),
          parent_id(-1) {}
    bool operator<(const Node &node) const {
      return g + h * 1.2 + extra_cost < node.g + node.h * 1.2 + node.extra_cost;
    }
    bool operator>(const Node &node) const {
      return g + h * 1.2 + extra_cost > node.g + node.h * 1.2 + node.extra_cost;
    }
    bool operator<=(const Node &node) const {
      return g + h * 1.2 + extra_cost <=
             node.g + node.h * 1.2 + node.extra_cost;
    }
    bool operator>=(const Node &node) const {
      return g + h * 1.2 + extra_cost >=
             node.g + node.h * 1.2 + node.extra_cost;
    }

    bool operator==(const Node &node) const {
      return x == node.x && y == node.y;
    }

    bool operator!=(const Node &node) const {
      return x != node.x || y != node.y;
    }
  };

  struct RobotInfo {
    int x;
    int y;
    // int team_id;
    // int enemy_id;
    double recv_time;  // (单位: s)
    RobotInfo(int x, int y,
              // int team_id, int enemy_id,
              double recv_time)
        : x(x),
          y(y),
          // team_id(team_id),
          // enemy_id(enemy_id),
          recv_time(recv_time) {}
    RobotInfo()
        : x(-1),
          y(-1),
          //  team_id(-1), enemy_id(-1),
          recv_time(-1) {}
  };

 private:
  int map_width;
  int map_height;
  std::unique_ptr<uchar[]> map;
  std::unique_ptr<std::unique_ptr<cv::Vec3d[]>[]> slope_map;
  std::unique_ptr<uint8_t[]> con_map;
  std::unique_ptr<uint32_t[]> towords_dis_map;

  int self_radius;
  int enemy_radius;

  bool enable_dynamic_obstacle = true;

  static const int army_type_num = 8;
  RobotInfo
      robot_info[army_type_num * 2];  // 0-red1 1-red2 ... 8-blue1 9-blue2 ...
  float robot_info_time_out = 5;      // (单位: s)

  int local_planner_area = 50;
  bool local_planner_enable = false;
  int local_planner_radius = 7;

  double dis_per_pixel_;
  double max_speed;
  double max_acc;
  double max_speed_in_slope;
  double max_turn_speed;

  double astarTimeOutTime;

  float timeTickAstar;

 public:
  AStar();

  /*******************************
   * @name find_path
   * @brief 前端A*算法寻路
   * @param [in] start 起点
   * @param [in] end 终点
   * @param [in] cnt_time 当前时间 (单位: s)
   * @param [out] fix_start 是否修正起点
   * @param [in] kExtraCost 与障碍物距离的额外代价
   * @param [in] step 寻路步长
   * @param [in] resolve_rate 寻找路径点的分辨率(在可选点中每多少个点取一个)
   */
  std::vector<Node> find_path(Node start, Node end, double cnt_time,
                              bool &fix_start, float kExtraCost = 1,
                              int step = 1, int resolve_rate = 1);

  /*******************************
   * @name hybrid_find_path
   * @brief 混合A*算法寻路
   * @param [in] start 起点
   * @param [in] end 终点
   * @param [in] cnt_time 当前时间 (单位: s)
   * @param [in] cnt_tick_cost_time 当前每帧花费的时间 (单位: s)
   * @param [in] on_end 目标点是否为终点(是否约束目标点速度为0)
   * @param [out] fix_start 是否修正起点
   * @param [in] kExtraCost 与障碍物距离的额外代价
   * @param [in] resolve_rate 寻找路径点的分辨率(在可选点中每多少个点取一个)
   */
  std::vector<Node> hybrid_find_path(Node start, Node end, double cnt_time,
                                     double cnt_tick_cost_time, bool on_end,
                                     bool &fix_start, float kExtraCost = 1,
                                     int resolve_rate = 1);

  float calc_h(Node cnt, Node end);

  Node find_nearest_available_pos(Node &pos_in_obstacle, int radius,
                                  float cnt_time);

  float get_min_dis(Node node, int radius, float cnt_time);

  uint32_t get_towords_dis(Node pos, double angle, int radius, float cnt_time);

  void fix_speed_on_slope(Node &pos, double &speed, double &angle);

  void set_enemy_pos(RobotInfo enemy_pos, int enemy_id) {
    this->robot_info[enemy_id] = enemy_pos;
  }

  void local_decision(Node &cnt, float kExtraCost = 1, int step = 1,
                      int resolve_rate = 1);

  std::vector<double> calc_available_speed(double cnt_speed, double theta_diff,
                                           double max_speed_diff);

  bool robot_is_connect(Node self, int robot_id, float cnt_time);

  inline bool is_direct_connection(Node start, Node end,
                                   bool consider_robot = false);

  Node local_planner(Node cnt, Node target, float cnt_time);

  float get_decision_cost(Node pos, float cnt_time);

  uchar &operator()(int x, int y) { return map[x + y * map_width]; }
  uchar &operator()(Node node) {
    return map[node.x_() + node.y_() * map_width];
  }

  void set_dynamic_obstacle_enable(bool enable) {
    enable_dynamic_obstacle = enable;
  }
};
}  // namespace navigation
#endif