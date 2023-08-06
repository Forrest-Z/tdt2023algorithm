#include "astar_frontend.h"

#include <omp.h>
#include <opencv2/core/cvdef.h>
#include <roborts_utils/base_msg.h>
#include <roborts_utils/base_param.h>
#include <roborts_utils/base_toolkit.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <vector>

namespace navigation {
AStar::AStar() {
  LoadParam::ReadParam("navigation_map", "dis_per_pixel", dis_per_pixel_);
  LoadParam::ReadParam("navigation", "self_radius", self_radius);
  self_radius /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "enemy_radius", enemy_radius);
  enemy_radius /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "perception_info_time_out",
                       robot_info_time_out);

  LoadParam::ReadParam("navigation", "local_planner_area", local_planner_area);
  local_planner_area /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "local_planner_enable",
                       local_planner_enable);
  LoadParam::ReadParam("navigation", "local_planner_radius",
                       local_planner_radius);
  LoadParam::ReadParam("navigation", "max_speed", max_speed);
  max_speed /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "max_acc", max_acc);
  LoadParam::ReadParam("navigation", "timeTickAstar", timeTickAstar);
  LoadParam::ReadParam("navigation", "astarTimeOutTime", astarTimeOutTime);
  LoadParam::ReadParam("navigation", "max_turn_speed", max_turn_speed);
  LoadParam::ReadParam("navigation", "max_speed_in_slope", max_speed_in_slope);
  max_acc /= dis_per_pixel_;
  map_width = 0;
  map_height = 0;
  map = nullptr;

  std::string map_name;
  LoadParam::ReadParam("navigation", "map_name", map_name);

  auto map = cv::imread(
      "install/roborts_navigation/share/roborts_navigation/cost_map/"
      "min_dis_map/" +
          map_name + ".png",
      cv::IMREAD_GRAYSCALE);
  this->map_width = map.cols;
  this->map_height = map.rows;
  this->map = std::make_unique<uchar[]>(map_width * map_height);
  for (int i = 0; i < map_width; ++i) {
    for (int j = 0; j < map_height; ++j) {
      this->map[i + j * map_width] = map.at<uchar>(j, i);
    }
  }
  size_t size = map_width * map_height;
  if (size == 0) {
    TDT_FATAL("地图尺寸为0.");
  }

  size = (size * size - 1) / 8 + 1;
  con_map = std::make_unique<uint8_t[]>(size);
  std::fstream f;

  f.open("install/roborts_navigation/share/roborts_navigation/con_map/" +
             map_name + ".map",
         std::ios::in | std::ios::binary);
  if (f.is_open()) {
    f.read((char *)con_map.get(), size);
    f.close();
  } else {
    TDT_FATAL("无法读取导航连通图.");
  }

  size = map_width * map_height * 8;
  towords_dis_map = std::make_unique<uint32_t[]>(size);
  f = std::fstream();
  f.open(
      "install/roborts_navigation/share/roborts_navigation/cost_map/"
      "towords_dis_map/" +
          map_name + ".map",
      std::ios::in | std::ios::binary);
  if (f.is_open()) {
    f.read((char *)towords_dis_map.get(), size * sizeof(uint32_t));
    f.close();
  } else {
    TDT_FATAL("无法读取导航方向距离图.");
  }

  auto slope_map = cv::imread(
      "install/roborts_navigation/share/roborts_navigation/cost_map/"
      "slope_map/" +
      map_name + ".png");
  size = slope_map.cols * slope_map.rows;
  if (size == 0) {
    TDT_FATAL("坡度地图尺寸为0.");
  }
  this->slope_map =
      std::make_unique<std::unique_ptr<cv::Vec3d[]>[]>(slope_map.cols);
  for (int i = 0; i < slope_map.cols; ++i) {
    this->slope_map[i] = std::make_unique<cv::Vec3d[]>(slope_map.rows);
    for (int j = 0; j < slope_map.rows; ++j) {
      this->slope_map[i][j] = slope_map.at<cv::Vec3d>(j, i);
    }
  }
}

std::vector<AStar::Node> AStar::hybrid_find_path(
    Node start, Node end, double cnt_time, double cnt_tick_cost_time,
    bool on_end, bool &fix_start, float kExtraCost, int resolve_rate) {
  auto time = tdttoolkit::Time::GetTimeNow();
  // start.speed_x = 20;
  // start.speed_y = 10;
  if (map == nullptr) {
    TDT_ERROR("Map is not initialized.");
    return std::vector<Node>();
  }
  if (end.x < self_radius || end.x >= (map_width - self_radius) ||
      end.y < self_radius || end.y >= (map_height - self_radius)) {
    TDT_ERROR("End point is out of map.");
    // end = find_nearest_available_pos(end, self_radius, cnt_time);
  }
  if (map[end.x_() + end.y_() * map_width] <= self_radius) {
    TDT_WARNING("End point is in obstacle.");
    // end = find_nearest_available_pos(end, self_radius, cnt_time);
  }
  end = find_nearest_available_pos(end, self_radius, cnt_time);
  fix_start = false;
  // Node start_tmp = start;
  if (map[start.x_() + start.y_() * map_width] <= self_radius) {
    TDT_WARNING("Start point is in obstacle.");
    // start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
    // fix_start = true;
  }
  if (start.x_() < 0 || start.x_() >= map_width || start.y_() < 0 ||
      start.y_() >= map_height) {
    TDT_WARNING("Start point is out of map.");
    // start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
    // fix_start = true;
  }
  Node start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
  std::vector<Node> start_path;
  if (start_tmp != start) {
    fix_start = true;
    auto theta =
        atan2(start_tmp.y_() - start.y_(), start_tmp.x_() - start.x_());
    auto dis = sqrt(pow(start_tmp.y_() - start.y_(), 2) +
                    pow(start_tmp.x_() - start.x_(), 2));
    auto step = dis / max_speed;
    auto speed_x = max_speed * cos(theta);
    auto speed_y = max_speed * sin(theta);
    for (int i = 0; i < step; ++i) {
      start_path.push_back(Node(start.x_() + speed_x * i,
                                start.y_() + speed_y * i, speed_x, speed_y, 0));
    }
  }
  std::vector<Node> path;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

  // bool visited[map_width][map_height] = {0}; // ISO C++ forbids this behavior
  // (although gcc allows it)
  std::unique_ptr<std::unique_ptr<size_t[]>[]> visited(
      new std::unique_ptr<size_t[]>[map_width]);
  for (size_t i = 0; i < map_width; ++i) {
    visited[i] = std::make_unique<size_t[]>(map_height);
  }
  std::vector<Node> visited_list;
  start_tmp.parent_id = -1;
  start_tmp.g = 0;
  start_tmp.h = calc_h(start_tmp, end);
  float start_dis = get_min_dis(start_tmp, self_radius, cnt_time);
  start_tmp.extra_cost = kExtraCost * -log(start_dis / 2);
  open_list.push(start_tmp);
  Node nearest_node = start_tmp;
  int time_judge_times = 0;
  while (!open_list.empty()) {
    if (time_judge_times > 1e3) {
      if (tdttoolkit::Time::GetTimeNow() - time > astarTimeOutTime) {
        TDT_WARNING("AStar time out.");
        break;
      }
      time_judge_times = 0;
    }
    time_judge_times++;
    Node current = open_list.top();
    open_list.pop();
    double time_tick = timeTickAstar;
    if (current.parent_id == -1) {
      // time_tick = cnt_tick_cost_time;
      // if (time_tick > 2 * timeTickAstar) {
      //   time_tick = 2 * timeTickAstar;
      // }
      // if (time_tick < 0.5 * timeTickAstar) {
      //   time_tick = 0.5 * timeTickAstar;
      // }
    }
    double max_speed_diff = time_tick * max_acc;
    // TDT_DEBUG_() << "chose point: x: " << current.x << " y:" << current.y
    //              << " speed_x: " << current.speed_x
    //              << " speed_y:" << current.speed_y
    //              << " h:" << calc_h(current, end) << "    g:" << current.g
    //              << "    extra_cost:" << current.extra_cost;
    auto dis = std::sqrt(pow(current.x - end.x, 2) + pow(current.y - end.y, 2));
    if (current.h + dis / max_speed * 3 + current.extra_cost +
            current.g * 0.25 <
        nearest_node.h +
            std::sqrt(std::pow(end.x - nearest_node.x, 2) +
                      std::pow(end.y - nearest_node.y, 2)) /
                max_speed * 3 +
            nearest_node.extra_cost + nearest_node.g * 0.25) {
      nearest_node = current;
    }
    if (isinf(current.extra_cost) || isnan(current.extra_cost)) {
      break;
    }
    if (isinf(current.h) || isnan(current.h)) {
      break;
    }
    if (visited[current.x_()][current.y_()] > 0 &&
        current >= visited_list[visited[current.x_()][current.y_()] - 1]) {
      continue;
    }
    visited_list.push_back(current);
    visited[current.x_()][current.y_()] = visited_list.size();

    auto cnt_speed_toword = current.speed_x == 0
                                ? CV_PI / 2
                                : atan2(current.speed_y, current.speed_x);
    auto cnt_speed = std::sqrt(current.speed_x * current.speed_x +
                               current.speed_y * current.speed_y);
    // 判断是否能直线到达终点
    if (is_direct_connection(current, end, true)) {
      auto theta = atan2(end.y - current.y, end.x - current.x);
      auto speeds = calc_available_speed(cnt_speed, cnt_speed_toword - theta,
                                         max_speed_diff);
      double speed = -1;
      if (speeds.size() == 2) {
        speed = std::max(speeds[0], speeds[1]);
      } else if (speeds.size() == 1) {
        speed = speeds[0];
      }
      if (speed >= 0) {
        auto speed_down_dis = speed * speed / max_acc / 2;
        if (speed_down_dis <= dis) {
          double total_time = 0, speed_up_time = 0, speed_keep_time = 0,
                 speed_down_time = 0;
          double cnt_max_speed = 0;
          if (on_end) {
            auto speed_up_dis = (dis - speed_down_dis) / 2;
            cnt_max_speed =
                std::sqrt(speed * speed + 2 * max_acc * speed_up_dis);
            speed_keep_time = 0.0;
            if (cnt_max_speed > max_speed) {
              cnt_max_speed = max_speed;
              speed_up_dis =
                  (max_speed * max_speed - speed * speed) / 2 / max_acc;
              speed_keep_time =
                  (dis - speed_up_dis * 2 - speed_down_dis) / max_speed;
            }
            speed_up_time = (cnt_max_speed - speed) / max_acc;
            speed_down_time = speed / max_acc;
          } else {
            speed_down_dis = 0;
            cnt_max_speed = std::sqrt(speed * speed + 2 * max_acc * dis);
            speed_keep_time = 0.0;
            if (cnt_max_speed > max_speed) {
              cnt_max_speed = max_speed;
              auto speed_up_dis =
                  (max_speed * max_speed - speed * speed) / 2 / max_acc;
              speed_keep_time = (dis - speed_up_dis) / max_speed;
            }
            speed_up_time = (cnt_max_speed - speed) / max_acc;
            speed_down_time = 0;
          }
          total_time = speed_up_time * 2 + speed_keep_time + speed_down_time;
          std::vector<Node> tmp_path;
          tmp_path.push_back(current);
          for (double t = time_tick; t <= total_time; t += time_tick) {
            double new_speed = 0;
            if (on_end) {
              if (t < speed_up_time) {
                new_speed = speed + max_acc * t;
              } else if (t < speed_up_time + speed_keep_time) {
                new_speed = cnt_max_speed;
              } else {
                new_speed = cnt_max_speed -
                            max_acc * (t - speed_up_time - speed_keep_time);
              }
            } else {
              if (t < speed_up_time) {
                new_speed = speed + max_acc * t;
              } else {
                new_speed = cnt_max_speed;
              }
            }
            float x = tmp_path.back().x + new_speed * cos(theta) * time_tick;
            float y = tmp_path.back().y + new_speed * sin(theta) * time_tick;
            if (x < 0 || x >= map_width || y < 0 || y >= map_height) {
              // TDT_DEBUG("On check: Out Map");
              goto NO_END;
            }
            auto cnt_dis = get_min_dis(Node(x, y), self_radius, cnt_time);
            if (cnt_dis < self_radius) {
              // TDT_DEBUG("On check: Obstracle");
              goto NO_END;
            }
            Node new_node(x, y, new_speed * cos(theta), new_speed * sin(theta),
                          theta, 0, 0, 0, tmp_path.size() - 1);
            tmp_path.push_back(new_node);
          }
          if (tmp_path.back() != end) {
            tmp_path.push_back(end);
          }
          for (int i = tmp_path.size() - 1; i >= 0; i--) {
            path.push_back(tmp_path[i]);
          }
          while (current.parent_id != -1) {
            current = visited_list[current.parent_id];
            path.push_back(current);
            // TDT_DEBUG_() << "ans path: " << current.x << " y:" << current.y
            //              << " speed_x: " << current.speed_x
            //              << " speed_y:" << current.speed_y
            //              << " h:" << calc_h(current, end) << "    g:" <<
            //              current.g
            //              << "    extra_cost:" << current.extra_cost;
          }
          if (fix_start) {
            for (int i = start_path.size() - 1; i >= 0; --i)
              path.push_back(start_path[i]);
          }
          std::reverse(path.begin(), path.end());
          return path;
        }
      }
    }
  NO_END:
    auto available_angle = 0.0;
    if (cnt_speed <= max_speed_diff) {
      available_angle = CV_PI;
    } else {
      available_angle = atan(max_speed_diff / cnt_speed);
    }
    available_angle = std::min(available_angle, max_turn_speed * time_tick);

    auto cnt_obstra_dis = get_min_dis(current, self_radius, cnt_time);
    for (int i = 0; i <= resolve_rate; i++) {
      for (int f = 0; f < 2; f++) {
        auto theta = cnt_speed_toword +
                     available_angle * i / resolve_rate * (f ? -1 : 1);
        auto theta_diff =
            fabs(tdttoolkit::AngleLimit(fabs(theta - cnt_speed_toword)));

        auto turn_time = theta_diff / max_turn_speed;

        auto speeds =
            calc_available_speed(cnt_speed, theta_diff, max_speed_diff);
        // TDT_DEBUG("Cnt speed: %f, theta_diff: %f, Calc speed: %f", cnt_speed,
        //           fabs(theta - cnt_speed_toword), speed);
        auto toword_dis = get_towords_dis(
            current, tdttoolkit::AngleLimit(theta, 1), self_radius, cnt_time);
        auto allowed_speed = std::sqrt(toword_dis * max_acc);
        for (auto &speed : speeds) {
          if (speed < 0) {
            continue;
          }

          if (speed > allowed_speed) {
            auto speed_diff_with_allowed_speed =
                std::sqrt(std::pow(speed * cos(cnt_speed_toword) -
                                       allowed_speed * cos(theta),
                                   2) +
                          std::pow(speed * sin(cnt_speed_toword) -
                                       allowed_speed * sin(theta),
                                   2));
            if (speed_diff_with_allowed_speed > max_speed_diff)
              continue;
            else {
              speed = allowed_speed;
            }
          }
          float x = current.x + speed * cos(theta) * (time_tick - turn_time);
          float y = current.y + speed * sin(theta) * (time_tick - turn_time);

          if (x < 0 || x >= map_width || y < 0 || y >= map_height) {
            continue;
          }

          if (map[(int)x + (int)y * map_width] == 0) continue;
          float cnt_cost = time_tick;
          // float cnt_cost = cnt_speed * time_tick;
          // float cnt_cost = std::sqrt(i * i + j * j);
          float next_obstra_dis =
              get_min_dis(Node(x, y), self_radius, cnt_time);
          next_obstra_dis = next_obstra_dis < 0 ? 0 : next_obstra_dis;

          // Node neighbor(
          //     x, y, 0, 0, current.g + cnt_cost,
          //     calc_h(Node(x, y, speed * cos(theta), speed * sin(theta)),
          //     -log(next_obstra_dis / 2) * kExtraCost, visited_list.size() -
          //     1);
          Node neighbor(
              x, y, speed * cos(theta), speed * sin(theta), theta,
              current.g + cnt_cost,
              calc_h(Node(x, y, speed * cos(theta), speed * sin(theta), theta),
                     end),
              -log(next_obstra_dis / 2) * kExtraCost, visited_list.size() - 1);
          // if (visited[x][y]) {
          //   continue;
          // }
          if (visited[neighbor.x_()][neighbor.y_()] > 0 &&
              neighbor >=
                  visited_list[visited[neighbor.x_()][neighbor.y_()] - 1]) {
            continue;
          }

          open_list.push(neighbor);
        }
      }
    }
  }
  TDT_ERROR("Unreachable.");
  auto current = nearest_node;
  while (current.parent_id != -1) {
    path.push_back(current);
    current = visited_list[current.parent_id];
  }
  path.push_back(current);
  if (fix_start) path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
  // return std::vector<Node>();
}

std::vector<AStar::Node> AStar::find_path(Node start, Node end, double cnt_time,
                                          bool &fix_start, float kExtraCost,
                                          int step, int resolve_rate) {
  if (map == nullptr) {
    TDT_ERROR("Map is not initialized.");
    return std::vector<Node>();
  }
  if (end.x < self_radius || end.x >= (map_width - self_radius) ||
      end.y < self_radius || end.y >= (map_height - self_radius)) {
    TDT_ERROR("End point is out of map.");
    // end = find_nearest_available_pos(end, self_radius, cnt_time);
  }
  if (map[end.x_() + end.y_() * map_width] <= self_radius) {
    TDT_WARNING("End point is in obstacle.");
    // end = find_nearest_available_pos(end, self_radius, cnt_time);
  }
  end = find_nearest_available_pos(end, self_radius, cnt_time);
  fix_start = false;
  // Node start_tmp = start;
  if (map[start.x_() + start.y_() * map_width] <= self_radius) {
    TDT_WARNING("Start point is in obstacle.");
    // start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
    // fix_start = true;
  }
  if (start.x < 0 || start.x >= map_width || start.y < 0 ||
      start.y >= map_height) {
    TDT_WARNING("Start point is out of map.");
    // start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
    // fix_start = true;
  }
  Node start_tmp = find_nearest_available_pos(start, self_radius, cnt_time);
  if (start_tmp != start) {
    fix_start = true;
  }
  std::vector<Node> path;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

  // bool visited[map_width][map_height] = {0}; // ISO C++ forbids this behavior
  // (although gcc allows it)
  std::unique_ptr<std::unique_ptr<bool[]>[]> visited(
      new std::unique_ptr<bool[]>[map_width]);
  for (int i = 0; i < map_width; ++i) {
    visited[i] = std::make_unique<bool[]>(map_height);
  }

  std::vector<Node> visited_list;
  visited[start_tmp.x_()][start_tmp.y_()] = true;
  start_tmp.parent_id = -1;
  start_tmp.g = 0;
  start_tmp.h = abs(start_tmp.x - end.x) + abs(start_tmp.y - end.y);
  int start_dis = get_min_dis(start_tmp, self_radius, cnt_time);
  start_tmp.extra_cost = kExtraCost * -log(start_dis / 2);
  open_list.push(start_tmp);
  Node nearest_node = start_tmp;

  while (!open_list.empty()) {
    Node current = open_list.top();
    if (current.h * 4 + current.extra_cost + current.g * 0.25 <
        nearest_node.h * 4 + nearest_node.extra_cost + nearest_node.g * 0.25) {
      nearest_node = current;
    }
    if (isinf(current.extra_cost) || isnan(current.extra_cost)) {
      TDT_ERROR("Unreachable.");
      current = nearest_node;
      while (current.parent_id != -1) {
        path.push_back(current);
        current = visited_list[current.parent_id];
      }
      path.push_back(current);
      if (fix_start) path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }
    visited_list.push_back(current);
    open_list.pop();
    if (pow(current.x - end.x, 2) + pow(current.y - end.y, 2) < step) {
      if (current.x != end.x || current.y != end.y) {
        path.push_back(end);
      }
      while (current.parent_id != -1) {
        path.push_back(current);
        current = visited_list[current.parent_id];
      }
      path.push_back(current);
      if (fix_start) path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }
    for (int i = -step; i <= step; i += resolve_rate) {
      for (int f = 0; f < 2; f++) {
        int j = (sqrt(step * step - i * i)) * (f ? -1 : 1);
        int x = current.x + i;
        int y = current.y + j;
        if (x < 0 || x >= map_width || y < 0 || y >= map_height) {
          continue;
        }
        if (visited[x][y]) {
          continue;
        }
        if (map[x + y * map_width] == 0) continue;
        float cnt_cost = sqrt(i * i + j * j);
        int cnt_dis = get_min_dis(Node(x, y), self_radius, cnt_time);
        cnt_dis = cnt_dis < 0 ? 0 : cnt_dis;
        // Node  neighbor(x, y, 0, 0, 0, current.g + i + j + std::min(i, j) *
        // (sqrt(2) - 2), std::abs(x - end.x) + std::abs(y - end.y) +
        // std::min(std::abs(x - end.x), std::abs(y - end.y)) * (sqrt(2) -
        // 2), -log(map[x + y * map_width]) * kExtraCost,
        // visited_list.size() - 1);
        Node neighbor(
            x, y, 0, 0, 0, current.g + cnt_cost,
            sqrt((y - end.y) * (y - end.y) + (x - end.x) * (x - end.x)),
            -log(cnt_dis / 2) * kExtraCost, visited_list.size() - 1);

        visited[x][y] = true;
        open_list.push(neighbor);
      }
    }
  }
  return path;
}

float AStar::calc_h(Node cnt, Node end) {
  // eular
  // return sqrt((cnt.y - end.y) * (cnt.y - end.y) +
  //             (cnt.x - end.x) * (cnt.x - end.x));

  // dubins
  auto theta1 = atan2(cnt.speed_y, cnt.speed_x);
  auto theta2 = atan2(end.y - cnt.y, end.x - cnt.x);
  auto theta_diff = fabs(tdttoolkit::AngleLimit(theta1 - theta2));
  auto speed = sqrt(cnt.speed_x * cnt.speed_x + cnt.speed_y * cnt.speed_y);
  auto R = (cnt.speed_x * cnt.speed_x + cnt.speed_y * cnt.speed_y) / max_acc;
  auto dis = sqrt((cnt.y - end.y) * (cnt.y - end.y) +
                  (cnt.x - end.x) * (cnt.x - end.x));
  if (speed == 0 || theta_diff == 0) {
    auto speed_up_time = (max_speed - speed) / max_acc;
    auto speed_length =
        speed_up_time * speed + max_acc * speed_up_time * speed_up_time / 2;
    if (speed_length < dis) {
      auto l2_time = (dis - speed_length) / max_speed + speed_up_time;
      return l2_time;
    } else {
      auto cnt_v_max = sqrt(2 * max_acc * dis + speed * speed);
      auto l2_time = (cnt_v_max - speed) / max_acc;
      return l2_time;
    }
  }
  if (2 * R > dis) {
    auto t = 2 * R * sin(theta_diff);
    if (t >= dis) {
      return std::numeric_limits<float>::infinity();
    }
  }
  if (theta_diff > CV_PI / 2) {
    auto t = std::sqrt(R * R + dis * dis -
                       2 * R * dis * cos(theta_diff - CV_PI / 2));
    auto alpha2 = asin(R / t);
    auto alpha1 = asin(R / t * sin(theta_diff - CV_PI / 2));
    auto alpha = alpha1 + alpha2;
    if (alpha > CV_PI) alpha -= CV_PI;
    auto l1 = R * (alpha + theta_diff);
    auto l2 = std::sqrt(t * t - R * R);
    if (alpha1 < 0 || alpha2 < 0 || l1 < 0 || l2 < 0 || t < R) {
      // TDT_ERROR("calc error1");
      return std::numeric_limits<float>::infinity();
    }
    auto speed_up_time = (max_speed - speed) / max_acc;
    auto speed_length =
        speed_up_time * speed + max_acc * speed_up_time * speed_up_time / 2;
    if (speed_length < l2) {
      auto l2_time = (l2 - speed_length) / max_speed + speed_up_time;
      return speed > 0 ? l1 / speed + l2_time : l2_time;
    } else {
      auto cnt_v_max = sqrt(2 * max_acc * l2 + speed * speed);
      auto l2_time = (cnt_v_max - speed) / max_acc;
      return speed > 0 ? l1 / speed + l2_time : l2_time;
    }
  } else {
    auto t = std::sqrt(R * R + dis * dis -
                       2 * R * dis * cos(CV_PI / 2 - theta_diff));
    auto l2 = std::sqrt(t * t - R * R);
    auto alpha2 = asin(l2 / t);
    auto alpha1 = asin(dis / t * sin(CV_PI / 2 - theta_diff));
    if (alpha1 < alpha2) alpha1 = CV_PI - alpha1;
    auto alpha = alpha1 - alpha2;
    auto l1 = R * alpha;
    if (alpha1 < 0 || alpha2 < 0 || l1 < 0 || l2 < 0 || t < R) {
      // TDT_ERROR("calc error2");
      return std::numeric_limits<float>::infinity();
    }
    auto speed_up_time = (max_speed - speed) / max_acc;
    auto speed_length =
        speed_up_time * speed + max_acc * speed_up_time * speed_up_time / 2;
    if (speed_length < l2) {
      auto l2_time = (l2 - speed_length) / max_speed + speed_up_time;
      return speed > 0 ? l1 / speed + l2_time : l2_time;
    } else {
      auto cnt_v_max = sqrt(2 * max_acc * l2 + speed * speed);
      auto l2_time = (cnt_v_max - speed) / max_acc;
      return speed > 0 ? l1 / speed + l2_time : l2_time;
    }
  }
}

AStar::Node AStar::find_nearest_available_pos(Node &pos_in_obstacle, int radius,
                                              float cnt_time) {
  int start_min_dis = get_min_dis(pos_in_obstacle, radius, cnt_time);
  if (start_min_dis > 1) {
    return pos_in_obstacle;
  }

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  // bool visited[map_width][map_height] = {0}; // ISO C++ forbids this behavior
  std::unique_ptr<std::unique_ptr<bool[]>[]> visited(
      new std::unique_ptr<bool[]>[map_width]);
  for (int i = 0; i < map_width; ++i) {
    visited[i] = std::make_unique<bool[]>(map_height);
  }

  std::vector<Node> visited_list;
  visited[pos_in_obstacle.x_()][pos_in_obstacle.y_()] = true;
  pos_in_obstacle.parent_id = -1;
  open_list.push(pos_in_obstacle);
  while (!open_list.empty()) {
    Node current = open_list.top();
    visited_list.push_back(current);
    open_list.pop();
    int min_dis = get_min_dis(current, radius, cnt_time);

    if (min_dis > 1) {
      auto theta =
          atan2(current.y - pos_in_obstacle.y, current.x - pos_in_obstacle.x);
      auto start_speed_towords =
          atan2(pos_in_obstacle.speed_y, pos_in_obstacle.speed_x);
      auto start_speed_size =
          sqrt(pos_in_obstacle.speed_x * pos_in_obstacle.speed_x +
               pos_in_obstacle.speed_y * pos_in_obstacle.speed_y);
      auto avai_speeds = calc_available_speed(
          start_speed_size,
          fabs(tdttoolkit::CalcAngleDifference(theta, start_speed_towords)),
          max_acc * timeTickAstar);
      auto speed = avai_speeds.size() > 0
                       ? avai_speeds.size() > 1
                             ? std::max(avai_speeds[0], avai_speeds[1])
                             : avai_speeds[0]
                       : 0;
      current.speed_x = speed * cos(theta);
      current.speed_y = speed * sin(theta);
      return current;
    }
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        int x = current.x + i;
        int y = current.y + j;
        if (x < 0 || x >= map_width || y < 0 || y >= map_height) {
          continue;
        }
        if (visited[x][y]) {
          continue;
        }
        float cost = sqrt(i * i + j * j);
        auto theta = atan2(y - pos_in_obstacle.y, x - pos_in_obstacle.x);
        auto neighbor =
            Node(x, y, pos_in_obstacle.speed_x, pos_in_obstacle.speed_y,
                 pos_in_obstacle.last_turn_angle, current.g + cost, 0, 0, 0);
        visited[x][y] = true;
        open_list.push(neighbor);
      }
    }
  }
  return pos_in_obstacle;
}

float AStar::get_min_dis(AStar::Node node, int radius, float cnt_time) {
  float min_dis = 0;
  if (node.x < 0 || node.x >= map_width || node.y < 0 || node.y >= map_height)
    min_dis = 0;
  else
    min_dis = map[node.x_() + node.y_() * map_width] - radius;
  if (enable_dynamic_obstacle) {
    for (int j = 0; j < army_type_num * 2; j++) {
      if (cnt_time - robot_info[j].recv_time < robot_info_time_out &&
          robot_info[j].recv_time > 0) {
        float enemy_dis =
            sqrt((robot_info[j].x - node.x) * (robot_info[j].x - node.x) +
                 (robot_info[j].y - node.y) * (robot_info[j].y - node.y));
        enemy_dis = enemy_dis - enemy_radius - radius;
        min_dis = std::min(min_dis, enemy_dis);
      }
    }
  }
  return min_dis;
}

uint32_t AStar::get_towords_dis(AStar::Node pos, double angle, int radius,
                                float cnt_time) {
  size_t index = (pos.x_() * map_height + pos.y_());
  index = index * 8 + (int)(angle / (M_PI_2) + 0.5);
  float min_dis = towords_dis_map[index];
  if (enable_dynamic_obstacle) {
    for (int j = 0; j < army_type_num * 2; j++) {
      if (cnt_time - robot_info[j].recv_time < robot_info_time_out &&
          robot_info[j].recv_time > 0) {
        auto enemy_angle =
            atan2(robot_info[j].y - pos.y, robot_info[j].x - pos.x);
        if (fabs(tdttoolkit::CalcAngleDifference(angle, enemy_angle)) <
            M_PI_4) {
          float enemy_dis =
              sqrt((robot_info[j].x - pos.x) * (robot_info[j].x - pos.x) +
                   (robot_info[j].y - pos.y) * (robot_info[j].y - pos.y));
          if (enemy_dis < enemy_radius) {
            enemy_dis /= enemy_radius;
          } else {
            enemy_dis = enemy_dis - enemy_radius;
          }
          min_dis = std::min(min_dis, enemy_dis);
        }
      }
    }
  }

  return std::fmax(min_dis - radius, 0);
}

std::vector<double> AStar::calc_available_speed(double cnt_speed,
                                                double theta_diff,
                                                double max_speed_diff) {
  // theta_diff = fabs(tdttoolkit::AngleLimit(theta_diff));
  std::vector<double> ret;
  if (theta_diff == 0) {
    ret.push_back(std::min(cnt_speed + max_speed_diff, max_speed));
    ret.push_back(std::max(cnt_speed - max_speed_diff, -max_speed));
    return ret;
  }
  auto sin_alpha =
      cnt_speed / max_speed_diff *
      sin(theta_diff);  //将换向过程抽象为三角形后，与当前速度的边对应的角的sin值
  if (fabs(sin_alpha) > 1) return ret;
  auto alpha = asin(sin_alpha);

  // if (theta_diff + alpha < CV_PI) {
  double speed1 =
      sin(CV_PI - theta_diff - alpha) * max_speed_diff / sin(theta_diff);
  std::min(speed1, max_speed);
  std::max(speed1, -max_speed);
  ret.push_back(speed1);
  // }
  // if (theta_diff + CV_PI - alpha < CV_PI) {
  double speed2 = sin(alpha - theta_diff) * max_speed_diff / sin(theta_diff);
  std::min(speed2, max_speed);
  std::max(speed2, -max_speed);
  if (speed2 != speed1) ret.push_back(speed2);
  // }

  // ret.push_back((speed1 + speed2) / 2);

  return ret;
}

bool AStar::robot_is_connect(AStar::Node self, int robot_id, float cnt_time) {
  if (cnt_time - robot_info[robot_id].recv_time < robot_info_time_out &&
      robot_info[robot_id].recv_time > 0) {
    if (!is_direct_connection(
            self, Node(robot_info[robot_id].x, robot_info[robot_id].y)))
      return false;
  };
  // self.x + b * self.y + c = 0
  // robot_info[robot_id].x + b * robot_info[robot_id].y + c = 0
  auto a = self.y - robot_info[robot_id].y;
  auto b = robot_info[robot_id].x - self.x;
  auto c = self.x * robot_info[robot_id].y - robot_info[robot_id].x * self.y;
  if (enable_dynamic_obstacle) {
    for (int i = 0; i < army_type_num * 2; i++) {
      if (cnt_time - robot_info[i].recv_time < robot_info_time_out &&
          robot_info[i].recv_time > 0) {
        if (i != robot_id &&
            is_direct_connection(self,
                                 Node(robot_info[i].x, robot_info[i].y)) &&
            is_direct_connection(
                Node(robot_info[i].x, robot_info[i].y),
                Node(robot_info[robot_id].x, robot_info[robot_id].y))) {
          if (fabs(a * robot_info[i].x + b * robot_info[i].y + c) <
              sqrt(a * a + b * b) * enemy_radius)
            return false;
        }
      }
    }
  }
  return true;
}

inline bool AStar::is_direct_connection(AStar::Node start, AStar::Node end,
                                        bool consider_robot) {
  size_t index = (start.x_() * map_height + start.y_());
  index = index * map_width * map_height + end.x_() * map_height + end.y_();
  size_t byte = index / 8;
  int bit = index % 8;
  bool ret = con_map[byte] & (1 << bit);
  if (ret) {
    if (consider_robot && enable_dynamic_obstacle) {
      auto theta = atan2(end.y_() - start.y_(), end.x_() - start.x_());
      for (int i = 0; i < army_type_num * 2; i++) {
        if (robot_info[i].recv_time > 0 &&
            robot_info[i].recv_time < robot_info_time_out) {
          auto dis = cos(theta) * (robot_info[i].x - start.x_()) +
                     sin(theta) * (robot_info[i].y - start.y_());
          if (fabs(dis) < self_radius + enemy_radius) {
            ret = false;
            break;
          }
        }
      }
    }
  }
  if (ret) {
    ret = ret;
  }
  return ret;
}

AStar::Node AStar::local_planner(Node cnt, AStar::Node target, float cnt_time) {
  for (int i = 0; i < army_type_num * 2; i++) {
    if (robot_info[i].recv_time > 0 &&
        cnt_time - robot_info[i].recv_time > robot_info_time_out) {
      if (is_direct_connection(cnt, Node(robot_info[i].x, robot_info[i].y))) {
        robot_info[i].recv_time = 0;
      }
    }
  }

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  // bool visited[map_width][map_height] = {0}; // ISO C++ forbids this
  // behavior
  std::unique_ptr<std::unique_ptr<bool[]>[]> visited(
      new std::unique_ptr<bool[]>[map_width]);
  for (int i = 0; i < map_width; ++i) {
    visited[i] = std::make_unique<bool[]>(map_height);
  }
  Node best_pos;
  float best_cost;

  std::vector<Node> visited_list;
  visited[target.x_()][target.y_()] = true;
  target.parent_id = -1;
  open_list.push(target);
  while (!open_list.empty()) {
    Node current = open_list.top();
    visited_list.push_back(current);
    open_list.pop();
    auto cost = get_decision_cost(current, cnt_time);

    if (cost < best_cost) {
      best_cost = cost;
      best_pos = current;
    }
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        int x = current.x + i;
        int y = current.y + j;
        if (x < 0 || x >= map_width || y < 0 || y >= map_height) {
          continue;
        }
        if (std::sqrt((x - target.x) * (x - target.x) +
                      (y - target.y) * (y - target.y)) > local_planner_area) {
          continue;
        }
        if (visited[x][y]) {
          continue;
        }
        float cost = sqrt(i * i + j * j);
        auto extracost = get_decision_cost(Node(i, j), cnt_time);
        Node neighbor(x, y, 0, 0, 0, current.g + cost, 0, extracost,
                      visited_list.size() - 1);
        visited[x][y] = true;
        open_list.push(neighbor);
      }
    }
  }
  return best_pos;
}

float AStar::get_decision_cost(AStar::Node pos, float cnt_time) {
  float points = 0;
  float cost = 0;
  for (int i = 0; i < army_type_num * 2; i++) {
    if (robot_info[i].recv_time > 0) {
      if (is_direct_connection(pos, Node(robot_info[i].x, robot_info[i].y))) {
        int k = 1;
        if (is_direct_connection(pos,
                                 Node(robot_info[i].x + local_planner_radius,
                                      robot_info[i].y))) {
          k *= 1.05;
        }
        if (is_direct_connection(pos,
                                 Node(robot_info[i].x - local_planner_radius,
                                      robot_info[i].y))) {
          k *= 1.05;
        }
        if (is_direct_connection(
                pos, Node(robot_info[i].x,
                          robot_info[i].y + local_planner_radius))) {
          k *= 1.05;
        }
        if (is_direct_connection(
                pos, Node(robot_info[i].x,
                          robot_info[i].y - local_planner_radius))) {
          k *= 1.05;
        }
        if (i % army_type_num == 0) {
          points = std::max(points, 10.0f * k);
          cost += 3.0f;
        }
        if (i % army_type_num == 2 || i % army_type_num == 3 ||
            i % army_type_num == 4) {
          points = std::max(points, 5.0f * k);
          cost += 5.0f;
        }
        if (i % army_type_num == 6) {
          points = std::max(points, 3.0f * k);
          cost += 10.0f;
        }
      }
    }
  }
  return cost - points;
}

}  // namespace navigation