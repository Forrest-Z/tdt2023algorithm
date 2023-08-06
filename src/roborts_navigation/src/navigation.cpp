#include "navigation.h"

#include <roborts_utils/base_blackboard.h>
#include <roborts_utils/base_class.h>
#include <roborts_utils/base_msg.h>
#include <roborts_utils/base_param.h>
#include <roborts_utils/base_taskscheduler.h>
#include <roborts_utils/base_toolkit.h>

#include <algorithm>
#include <chrono>
#include <localization_interface/msg/detail/lam2_nav__struct.hpp>
#include <rclcpp/qos.hpp>
#include <thread>
#include <vision_interface/msg/detail/manual_towords_control__struct.hpp>

#include "astar_frontend.h"
#include "rmw/qos_profiles.h"

Navigation::Navigation(std::shared_ptr<rclcpp::Node> ros_node) {
  LoadParam::ReadParam("navigation_map", "dis_per_pixel", dis_per_pixel_);
  LoadParam::ReadParam("navigation", "kExtraCostHybridAstar",
                       kExtraCostHybridAstar_);
  LoadParam::ReadParam("navigation", "kExtraCostAstar", kExtraCost_);
  LoadParam::ReadParam("navigation", "stepAstar", stepAstar_);
  LoadParam::ReadParam("navigation", "resolverRateAstar", resolverRateAstar_);
  LoadParam::ReadParam("navigation", "resolverRateHybridAstar",
                       resolverRateHybridAstar_);
  LoadParam::ReadParam("navigation", "order", order_);
  LoadParam::ReadParam("navigation", "resolverRatePolyfit",
                       resolverRatePolyfit_);
  LoadParam::ReadParam("navigation", "max_speed", max_speed_);
  max_speed_ /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "max_acc", max_acc_);
  max_acc_ /= dis_per_pixel_;
  LoadParam::ReadParam("navigation", "self_color", self_color_);
  LoadParam::ReadParam("navigation", "timeTickAstar", timeTickAstar);
  LoadParam::ReadParam("navigation", "astarLeadSizeForHybrid",
                       astarLeadSizeForHybrid);
  LoadParam::ReadParam("navigation", "local_planner_enable",
                       local_planner_enable);
  LoadParam::ReadParam("navigation_map", "lidar_map_start_x",
                       lidar_map_start_x_);
  LoadParam::ReadParam("navigation_map", "lidar_map_start_y",
                       lidar_map_start_y_);
  LoadParam::ReadParam("navigation_map", "lidar_map_start_yaw",
                       lidar_map_start_yaw_);

  auto json_node = LoadParam::GetJsonValue("navigation_map");
  auto manual_point_array = json_node["strategic_points"];
  for (auto &point : manual_point_array) {
    if (point[0].isArray()) {
      for (int i = 0; i < point.size(); i++) {
        intermediate_point_list_.push_back(
            RobotCommand(point[i][0].asDouble(), point[i][1].asDouble()));
        if (point.size() == 1) {
          intermediate_point_list_.back().target_type = 'A';
        } else {
          intermediate_point_list_.back().target_type = 'S';
        }
        if (i >= 1) {
          intermediate_point_list_[intermediate_point_list_.size() - 2]
              .next_command_id = intermediate_point_list_.size() - 1;
        }
      }
      strategic_point_list_.push_back(intermediate_point_list_.size() -
                                      point.size());
      if (point.size() > 1) {
        intermediate_point_list_.back().next_command_id =
            intermediate_point_list_.size() - point.size();
      }
    } else {
      intermediate_point_list_.push_back(
          RobotCommand(point[0].asDouble(), point[1].asDouble(), (int)'A'));
      strategic_point_list_.push_back(intermediate_point_list_.size() - 1);
    }
  }
  ros_node_ = ros_node;
  std::string map_name;
  LoadParam::ReadParam("navigation", "map_name", map_name);
  map_ = cv::imread(
      "install/roborts_navigation/share/roborts_navigation/cost_map/"
      "min_dis_map/" +
          map_name + ".png",
      cv::IMREAD_GRAYSCALE);
  polyfit_.initMap(map_.cols, map_.rows, map_.data);

  std::string command_map_name;
  LoadParam::ReadParam("navigation", "command_map_name", command_map_name);
  auto command_map = cv::imread(
      "install/roborts_navigation/share/roborts_navigation/command_map/img/" +
          command_map_name + ".png",
      cv::IMREAD_GRAYSCALE);
  command_map_ = std::make_unique<std::unique_ptr<uchar[]>[]>(command_map.cols);
  for (int i = 0; i < command_map.cols; i++) {
    command_map_[i] = std::make_unique<uchar[]>(command_map.rows);
    for (int j = 0; j < command_map.rows; j++) {
      command_map_[i][j] = command_map.at<uchar>(j, i);
    }
  }
  LoadParam::ReadParam("navigation_command_map", "dis_per_pixel",
                       command_map_dis_per_pixel_);

  localization_subscriber_ =
      ros_node_->create_subscription<localization_interface::msg::LAM2Nav>(
          "tdt_LAM_001",            // topic_name,
          rclcpp::SensorDataQoS(),  // QOS,
          std::bind(&Navigation::InitVelCallback, this, std::placeholders::_1));

  percep_subscriber_ =
      ros_node_->create_subscription<perception_interface::msg::Perception2Nav>(
          "Percep2Nav",             // topic_name,
          rclcpp::SensorDataQoS(),  // QOS,
          std::bind(&Navigation::PercepCallback, this, std::placeholders::_1));
  nav_command_subscriber_ =
      ros_node_->create_subscription<navigation_interface::msg::NavCommand>(
          "nav_command",  // topic_name,
          10,             // QOS,
          std::bind(&Navigation::NavCommandCallback, this,
                    std::placeholders::_1));
  nav_decision_command_subscriber_ =
      ros_node_
          ->create_subscription<navigation_interface::msg::NavDecisionCommand>(
              "nav_decision_command", 10,
              std::bind(&Navigation::NavDecisionCommandCallback, this,
                        std::placeholders::_1));
  usart_speed_subscriber_ =
      ros_node_->create_subscription<navigation_interface::msg::UsartSpeed2Nav>(
          "usart_speed2_nav", tdttoolkit::HighFrequencySensorDataQoS(),
          std::bind(&Navigation::UsartSpeedCallback, this,
                    std::placeholders::_1));

  manual_towords_control_publisher_ =
      ros_node_->create_publisher<vision_interface::msg::ManualTowordsControl>(
          "manual_towords", 10);
          
  controler_ = new navigation::Controler(dis_per_pixel_, &navigation_debug_);
  controler_->init();

  int initial_manual_id;
  LoadParam::ReadParam("navigation_map", "initial_manual_id",
                       initial_manual_id);
  if (initial_manual_id > 0 &&
      initial_manual_id <= strategic_point_list_.size()) {
    robot_command_ =
        intermediate_point_list_[strategic_point_list_[initial_manual_id - 1]];
  } else {
    stop_flag = true;
    controler_->set_arrive_status(true);
  }

#ifdef NAVIGATION_DEBUG
  navigation_debug_.init();
  // navigation_debug_.add_param("kExtraCost_", kExtraCost_, 50);
  navigation_debug_.add_param("stepAstar_", stepAstar_, 50);
  navigation_debug_.add_param("resolverRateAstar_", resolverRateAstar_, 50);
  navigation_debug_.add_param("order", order_, 50);
  navigation_debug_.add_param("resolverRatePolyfit", resolverRatePolyfit_, 50);
  navigation_debug_.add_param("max_speed_", max_speed_, 50);
  debug_subscriber_ =
      ros_node_
          ->create_subscription<navigation_interface::msg::NavigationDebug>(
              "navigation_debug",  // topic_name,
              3,                   // buff_size,
              std::bind(&Navigation::DebugInitVelCallback, this,
                        std::placeholders::_1));
#endif
  controler_->start();
}
Navigation::~Navigation() { ; }

void Navigation::start() {
  navigation_debug_.start();
  std::thread([this]() {
    rclcpp::spin(ros_node_);
    rclcpp::shutdown();
  }).detach();
  while (true) {
    Update();
  }
}

void Navigation::stop() {
  controler_->stop();
  navigation_debug_.stop();
}

void Navigation::Update() {
  try {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_cv_.wait(lock);  // 等待收到消息后信息更新发送信号
    lock.unlock();
    auto cnt_update_time =
        tdttoolkit::Time::GetTimeNow() / 1e6;  // 当前时间 单位s
    auto cnt_tick_cost_time =
        cnt_update_time - last_update_time;  // 当前每帧花费的时间 单位s
#if USE_HYBRID_ASTAR
    if (hybird_astar_find_answer_) {
      if (cnt_tick_cost_time < timeTickAstar) {
        std::this_thread::sleep_for(std::chrono::milliseconds(
            (int)((timeTickAstar - cnt_tick_cost_time) * 1e3 * 0.8)));
      }
      last_update_time = cnt_update_time;
    }
#else
    last_update_time = cnt_update_time;
#endif

    auto start_tick = cv::getTickCount();
    auto robot_status_ = this->robot_status_;
    //   if (robot_status_.position_x < 0 || robot_command_.goal_x < 0
    //   ||
    //       robot_status_.position_y < 0 || robot_command_.goal_y < 0
    //       || robot_status_.position_x > map_.cols ||
    //       robot_status_.position_y > map_.rows ||
    //       robot_command_.goal_x > map_.cols || robot_command_.goal_y
    //       > map_.rows || (fabs(robot_command_.goal_x -
    //       robot_status_.position_x)
    //       *
    //                dis_per_pixel_ <=
    //            5 &&
    //        fabs(robot_command_.goal_y - robot_status_.position_y) *
    //                dis_per_pixel_ <=
    //            5)) {
    // #ifdef NAVIGATION_DEBUG
    //     navigation_debug_.publish_frame(robot_status_.time);
    // #endif
    //     return;
    //   }
    tdttoolkit::NavStatus nav_status;
    nav_status.nav_pos =
        tdttoolkit::Vec2d(robot_status_.position_x, robot_status_.position_y);
    nav_status.nav_target =
        tdttoolkit::Vec2d(robot_command_.goal_x, robot_command_.goal_y);
    nav_status.specific_target = speicific_flag;
    nav_status.on_arrive = false;

    if ((robot_command_.goal_x == -1 && robot_command_.goal_y == -1) ||
        (robot_status_.position_x == -1 && robot_status_.position_y == -1)) {
#ifdef NAVIGATION_DEBUG
      navigation_debug_.publish_frame(robot_status_.time);
#endif
      // controler_->set_stop();
      tdttoolkit::BaseBlackboard::Set("nav_status", nav_status);
      return;
    }

    robot_status_.position_x = std::max(0.0, robot_status_.position_x);
    robot_status_.position_y = std::max(0.0, robot_status_.position_y);
    robot_status_.position_x =
        std::min((double)map_.cols - 1, robot_status_.position_x);
    robot_status_.position_y =
        std::min((double)map_.rows - 1, robot_status_.position_y);
    robot_command_.goal_x = std::max(0.0, robot_command_.goal_x);
    robot_command_.goal_y = std::max(0.0, robot_command_.goal_y);
    robot_command_.goal_x =
        std::min((double)map_.cols - 1, robot_command_.goal_x);
    robot_command_.goal_y =
        std::min((double)map_.rows - 1, robot_command_.goal_y);

    // navigation_debug_.update_position(
    //     tdttoolkit::Vec2f(robot_status_.position_x,
    //     robot_status_.position_y), robot_status_.yaw);
    navigation_debug_.update_target(
        tdttoolkit::Vec2f(robot_command_.goal_x, robot_command_.goal_y));

    int stepAstar = stepAstar_;
    bool fixed_start = false;
    TDT_INFO("velocity: x=%f,y=%f", robot_status_.velocity_x,
             robot_status_.velocity_y);
    auto controler_speed_output = controler_->GetOutputSpeed();
    auto cnt_speed =
        tdttoolkit::Vec2d(robot_status_.velocity_x, robot_status_.velocity_y);
    auto speed_diff = controler_speed_output - cnt_speed;

    if (speed_diff.length() < 1 * max_acc_) {
      cnt_speed = controler_speed_output;
    }

    auto path = astar_.find_path(
        navigation::AStar::Node((robot_status_.position_x),
                                (robot_status_.position_y), cnt_speed.x,
                                cnt_speed.y, robot_status_.turn_angle),
        navigation::AStar::Node((int)robot_command_.goal_x,
                                (int)robot_command_.goal_y),
        robot_status_.time, fixed_start, kExtraCost_, stepAstar,
        resolverRateAstar_);

    // if ((!fixed_start) && (path.size() <= 2 || (stop_flag && path.size() <=
    // 4))) {
    auto dis =
        std::sqrt((robot_command_.goal_x - robot_status_.position_x) *
                      (robot_command_.goal_x - robot_status_.position_x) +
                  (robot_command_.goal_y - robot_status_.position_y) *
                      (robot_command_.goal_y - robot_status_.position_y));
    if ((path.size() <= 2 &&
         cnt_speed.length() < max_acc_ * timeTickAstar * 2) ||
        (stop_flag /*&& path.size() <= 4)*/) ||
        (dis <= stepAstar * 2 &&
         cnt_speed.length() < max_acc_ * timeTickAstar * 2) ||
        (dis <= stepAstar * 4 && stop_flag)) {
      if (path.size() > 0) {
        if (local_planner_enable) {
          auto fixed_target = astar_.local_planner(
              navigation::AStar::Node(robot_status_.position_x,
                                      robot_status_.position_y),
              navigation::AStar::Node(robot_command_.goal_x,
                                      robot_command_.goal_y),
              robot_status_.time);
          ;
        }
#ifdef NAVIGATION_DEBUG
        navigation_debug_.set_frontnodes(path, robot_status_.time);
#endif
      }
#ifdef NAVIGATION_DEBUG
      navigation_debug_.publish_frame(robot_status_.time);
#endif
      nav_status.on_arrive = true;
      controler_->set_arrive_status(true);
      stop_flag = true;
      tdttoolkit::BaseBlackboard::Set("nav_status", nav_status);
      if (robot_command_.next_command_id != -1) {
        robot_command_ =
            intermediate_point_list_[robot_command_.next_command_id];
        stop_flag = false;
      }
      controler_->set_stop();

      return;
    }
    stop_flag = false;
    controler_->set_arrive_status(false);
    tdttoolkit::BaseBlackboard::Set("nav_status", nav_status);
#ifdef NAVIGATION_DEBUG
    navigation_debug_.set_frontnodes(path, robot_status_.time);
#endif
#if USE_HYBRID_ASTAR
    int end_index = astarLeadSizeForHybrid;
    bool on_end = false;
    if (path.size() < astarLeadSizeForHybrid) {
      end_index = path.size() - 1;
      on_end = true;
    }
    auto optimized_path = astar_.hybrid_find_path(
        path[0], path[end_index], robot_status_.time, cnt_tick_cost_time,
        on_end, fixed_start, kExtraCostHybridAstar_, resolverRateHybridAstar_);

    if (optimized_path.size() < 2) {
#ifdef NAVIGATION_DEBUG
      navigation_debug_.publish_frame(robot_status_.time);
#endif
      hybird_astar_find_answer_ = false;
      // controler_->set_stop();
      tdttoolkit::BaseBlackboard::Set("nav_status", nav_status);
      return;
    }
    hybird_astar_find_answer_ = true;

    auto backend_result = hybrid_astar_backend_.solve(
        optimized_path,
        std::make_pair(tdttoolkit::Vec2d(robot_status_.velocity_x,
                                         robot_status_.velocity_y),
                       tdttoolkit::Vec2d(0, 0)),
        order_, resolverRatePolyfit_, 50, max_speed_, fixed_start);
#else
    auto backend_result = direct_.solve(
        path,
        std::make_pair(tdttoolkit::Vec2d(robot_status_.velocity_x,
                                         robot_status_.velocity_y),
                       tdttoolkit::Vec2d(0, 0)),
        order_, resolverRatePolyfit_, 50, max_speed_, fixed_start);
#endif

    controler_->set_result(backend_result,
                           tdttoolkit::Time::GetTimeNow() / 1e6);
#ifdef NAVIGATION_DEBUG
    navigation_debug_.set_backendresult(backend_result, robot_status_.time);
    navigation_debug_.publish_frame(robot_status_.time);
#endif
    TDT_INFO("FPS: %f",
             cv::getTickFrequency() / (cv::getTickCount() - start_tick));
  } catch (...) {
    TDT_WARNING("Navigation Update Crash.");
    return;
  }
}

void Navigation::FixLidarPos(const double &pos_x, const double &pos_y,
                             const double &yaw, double &output_pos_x,
                             double &output_pos_y) {
  output_pos_x =
      (pos_y + lidar_map_start_x_ + 0.1 * cos(yaw)) * 100 / dis_per_pixel_;
  output_pos_y =
      (pos_x + lidar_map_start_y_ - 0.1 * sin(yaw)) * 100 / dis_per_pixel_;
  // tdttoolkit::Vec2f pos;
  // pos.x = output_pos_x;
  // pos.y = output_pos_y;
  // pos.rotate(-lidar_map_start_yaw_);
  // output_pos_x = pos.x;
  // output_pos_y = pos.y;
}

void Navigation::InitVelCallback(
    const localization_interface::msg::LAM2Nav &msg) {
  try {
    if (!msg.reliability) return;
    auto start_time = tdttoolkit::Time::GetTimeNow();
    tdttoolkit::Quaternion q;
    q.x = msg.pose.orientation.x;
    q.y = msg.pose.orientation.y;
    q.z = msg.pose.orientation.z;
    q.w = msg.pose.orientation.w;
    // q.x = msg.pose.orientation.x;
    RobotStatus cnt_robot_status_;
    cnt_robot_status_.time =
        tdttoolkit::Time::GetTimeByRosTime(msg.header.stamp) / 1e6;
    tdttoolkit::EulerAngles e = tdttoolkit::ToEulerAngles(q);
    e.yaw = e.yaw + lidar_map_start_yaw_ / 180 * CV_PI;
    controler_->set_robot_euler(e.yaw, e.pitch, e.roll, cnt_robot_status_.time);
    // TDT_INFO("yaw = %f",e.yaw);
    cnt_robot_status_.yaw = e.yaw;
    FixLidarPos(msg.pose.position.x, msg.pose.position.y, e.yaw,
                cnt_robot_status_.position_x, cnt_robot_status_.position_y);
    // cnt_robot_status_.velocity_x = 0;
    // cnt_robot_status_.velocity_y = 0;
    // TDT_INFO("pos_x = %lf pos_y = %lf", robot_status_.position_x,
    //          robot_status_.position_y);
    cnt_robot_status_.pos_valid = true;
    cnt_robot_status_.velocity_y = msg.linear_vel.x / dis_per_pixel_;
    cnt_robot_status_.velocity_x = msg.linear_vel.y / dis_per_pixel_;
    cnt_robot_status_.turn_angle = atan2(msg.linear_acc.y, msg.linear_acc.x);
    cnt_robot_status_.vel_valid = true;
    double time = tdttoolkit::Time::GetTimeNow() / 1e6;
    // if (!usart_speed_history_.empty()) {
    //   auto cnt_usart_speed = std::lower_bound(
    //       usart_speed_history_.begin(), usart_speed_history_.end(), time,
    //       [](const UsartSpeed &a, const double &b) { return a.time < b; });
    //   if (cnt_usart_speed == usart_speed_history_.end()) {
    //     cnt_usart_speed = usart_speed_history_.end() - 1;
    //   }
    //   auto usart_speed =
    //       tdttoolkit::Vec2d(cnt_usart_speed->speed_x,
    //       cnt_usart_speed->speed_y);
    //   usart_speed.rotate(-e.yaw);
    //   cnt_robot_status_.velocity_x = usart_speed.x;
    //   cnt_robot_status_.velocity_y = usart_speed.y;
    // }
    controler_->set_robot_pos(cnt_robot_status_.position_x,
                              cnt_robot_status_.position_y,
                              cnt_robot_status_.time);
    controler_->set_robot_speed(cnt_robot_status_.velocity_x,
                                cnt_robot_status_.velocity_y,
                                cnt_robot_status_.time);
    controler_->set_robot_speed(cnt_robot_status_.velocity_x,
                                cnt_robot_status_.velocity_y,
                                cnt_robot_status_.time);

    if (robot_status_history_.empty() ||
        cnt_robot_status_.time > robot_status_history_.back().time) {
      robot_status_history_.push_back(cnt_robot_status_);
    } else {
      auto it =
          std::lower_bound(robot_status_history_.begin(),
                           robot_status_history_.end(), cnt_robot_status_.time);
      if (it == robot_status_history_.end()) {
        it = robot_status_history_.end() - 1;
      }
      robot_status_history_.insert(it, cnt_robot_status_);
    }
    if (robot_status_history_.size() > cache_size) {
      robot_status_history_.pop_front();
    }

#ifdef NAVIGATION_DEBUG
    navigation_debug_.update_position(
        tdttoolkit::Vec2f(cnt_robot_status_.position_x,
                          cnt_robot_status_.position_y),
        cnt_robot_status_.yaw);
#endif
    robot_status_ = cnt_robot_status_;
    // Update();
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_cv_.notify_one();
  } catch (...) {
    TDT_WARNING("Localization Msg Callback Crash.");
    return;
  }
}

void Navigation::PercepCallback(
    const perception_interface::msg::Perception2Nav &msg) {
  auto time = tdttoolkit::Time::GetTimeByRosTime(msg.header.stamp) / 1e6;
  time = tdttoolkit::Time::GetTimeNow() / 1e6;
  auto target_status = robot_status_;
  if (!robot_status_history_.empty()) {
    auto it = std::lower_bound(robot_status_history_.begin(),
                               robot_status_history_.end(), time);
    if (it == robot_status_history_.end()) {
      it = robot_status_history_.end() - 1;
    }
    target_status = *it;
  }
  for (auto &robot : msg.robots) {
    tdttoolkit::Vec2f pos;
    pos.x = -robot.center.x;
    pos.y = robot.center.y;
    pos.rotate(-target_status.yaw);
    double x, y;
    y = pos.y / dis_per_pixel_ + target_status.position_y;
    x = pos.x / dis_per_pixel_ + target_status.position_x;
    astar_.set_enemy_pos(navigation::AStar::RobotInfo(x, y, time),
                         robot.class_id - 1);
    navigation_debug_.set_enemy_pos(navigation::AStar::RobotInfo(x, y, time),
                                    robot.class_id - 1);
  };
  // Update();
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wait_cv_.notify_one();
}

void Navigation::NavCommandCallback(
    const navigation_interface::msg::NavCommand &msg) {
  if (speicific_flag > 0) {
    TDT_WARNING("Skip Command by Speicific Target.");
    return;
  }
  auto new_robot_command_ = robot_command_;
  TDT_DEBUG_() << "recv command message:" << msg.target_type << " goal_x"
               << msg.manual_position_x << " goal_y:" << msg.manual_position_y;
  if (msg.manual_position_x >= 0 && msg.manual_position_y >= 0) {
    // TDT_DEBUG_() << "recv commander type:" << robot_command_.target_type
    //              << " goal_x" << robot_command_.goal_x
    //              << " goal_y:" << robot_command_.goal_y;
    auto map_x = msg.manual_position_x * 100 / dis_per_pixel_;
    if (self_color_ == 0) {
      map_x = (28 - msg.manual_position_x) * 100 / dis_per_pixel_;
    }
    auto map_y = (15 - msg.manual_position_y) * 100 / dis_per_pixel_;
    if (self_color_ == 0) {
      map_y = msg.manual_position_y * 100 / dis_per_pixel_;
    }

    int command_x = msg.manual_position_x * 100 / command_map_dis_per_pixel_;
    if (self_color_ == 0) {
      command_x =
          (28 - msg.manual_position_x) * 100 / command_map_dis_per_pixel_;
    }
    int command_y =
        (15 - msg.manual_position_y) * 100 / command_map_dis_per_pixel_;

    if (msg.target_type == 'A' || msg.target_type == 'D' ||
        msg.target_type == 'X') {
      new_robot_command_.goal_x = map_x;
      new_robot_command_.goal_y = map_y;
      new_robot_command_.target_type = msg.target_type;
      new_robot_command_.next_command_id = -1;
      if (new_robot_command_ == robot_command_) {
        return;
      }
      robot_command_ = new_robot_command_;
      controler_->set_target_type(new_robot_command_.target_type);
      if (new_robot_command_.target_type == 'D') {
        astar_.set_dynamic_obstacle_enable(false);
      } else {
        astar_.set_dynamic_obstacle_enable(true);
      }
      command_id++;
      stop_flag = false;
    } else if (msg.target_type == 'Z') {
      robot_command_.target_type = msg.target_type;
      controler_->set_target_type(robot_command_.target_type);
    } else if (msg.target_type == 'S') {
      auto id = command_map_[command_x][command_y] / 10;
      if (id > 0 && id <= strategic_point_list_.size()) {
        new_robot_command_ =
            intermediate_point_list_[strategic_point_list_[id - 1]];
        if (new_robot_command_ == robot_command_) {
          return;
        }
        robot_command_ = new_robot_command_;

        controler_->set_target_type(new_robot_command_.target_type);
        if (new_robot_command_.target_type == 'D') {
          astar_.set_dynamic_obstacle_enable(false);
        } else {
          astar_.set_dynamic_obstacle_enable(true);
        }
        command_id++;
        stop_flag = false;
      }
    } else if (msg.target_type == 'C') {
      auto yaw = atan2(map_y - robot_status_.position_y,
                       map_x - robot_status_.position_x);
      yaw = -yaw + lidar_map_start_yaw_ / 180 * CV_PI;
      auto yaw_diff = yaw - robot_status_.yaw;
      yaw_diff = tdttoolkit::AngleLimit(yaw_diff);
      vision_interface::msg::ManualTowordsControl manual_towords_control;
      manual_towords_control.header.stamp =
          tdttoolkit::Time::GetRosTimeByTime(robot_status_.time);
      manual_towords_control.header.frame_id = "manual_towords_control";
      manual_towords_control.yaw_diff = yaw_diff;
      manual_towords_control_publisher_->publish(manual_towords_control);
    } else if (msg.target_type == 'Q') {
      stop_flag = true;
      controler_->set_stop();
    } else {
      return;
    }
  } else {
    if (msg.marked_id > 0) {
      return;
    } else {
      TDT_ERROR("操作手手动标记机器人ID错误");
      return;
    }
  }
  TDT_DEBUG_() << "command type:" << robot_command_.target_type << " goal_x"
               << robot_command_.goal_x << " goal_y:" << robot_command_.goal_y;
  speicific_flag = tdttoolkit::NavStatus::SpecificTarget::NONE;
  last_recv_time_ = tdttoolkit::Time::GetTimeByRosTime(msg.header.stamp) / 1e6;
  // Update();
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wait_cv_.notify_one();
}

void Navigation::NavDecisionCommandCallback(
    const navigation_interface::msg::NavDecisionCommand &msg) {
  if (msg.specific_point_id > 0) {
    try {
      auto json_node = LoadParam::GetJsonValue("navigation_map");
      auto point_list = json_node["speicific_points"];
      if (msg.specific_point_id == 1) {
        if (point_list["supply"].isArray()) {
          TDT_WARNING("哨兵前往补给区");
          robot_command_.goal_x = json_node["supply"][0].asDouble();
          robot_command_.goal_y = json_node["supply"][1].asDouble();
          robot_command_.next_command_id = -1;
          stop_flag = false;
          speicific_flag = tdttoolkit::NavStatus::SpecificTarget::SUPPLY;
        }
      }
    } catch (...) {
      TDT_WARNING("NavDecisionCommandCallback Crash.");
      return;
    }
  } else {
    robot_command_.goal_x = msg.point_x;
    robot_command_.goal_y = msg.point_y;
    robot_command_.next_command_id = -1;
    stop_flag = false;
    speicific_flag = tdttoolkit::NavStatus::SpecificTarget::NONE;
    ;
  }
  // Update();
  last_recv_time_ = tdttoolkit::Time::GetTimeByRosTime(msg.header.stamp) / 1e6;
  wait_cv_.notify_one();
}

void Navigation::UsartSpeedCallback(
    const navigation_interface::msg::UsartSpeed2Nav &msg) {
  UsartSpeed usart_speed;
  usart_speed.time = tdttoolkit::Time::GetTimeByRosTime(msg.header.stamp) / 1e6;
  usart_speed.speed_y = 0 - msg.speed_x / 10 / dis_per_pixel_;
  usart_speed.speed_x = msg.speed_y / 10 / dis_per_pixel_;
  if (usart_speed_history_.empty() ||
      usart_speed.time > usart_speed_history_.back().time) {
    usart_speed_history_.push_back(usart_speed);
  } else {
    auto it = std::lower_bound(usart_speed_history_.begin(),
                               usart_speed_history_.end(), usart_speed.time);
    if (it == usart_speed_history_.end()) {
      it = usart_speed_history_.end() - 1;
    }
    usart_speed_history_.insert(it, usart_speed);
  }
  if (usart_speed_history_.size() > cache_size) {
    usart_speed_history_.pop_front();
  }
}

#ifdef NAVIGATION_DEBUG
void Navigation::DebugInitVelCallback(
    const navigation_interface::msg::NavigationDebug &msg) {
  if (msg.type == 0) {
    robot_status_.position_x = msg.position_x;
    robot_status_.position_y = msg.position_y;
    controler_->set_robot_pos(robot_status_.position_x,
                              robot_status_.position_y,
                              tdttoolkit::Time::GetTimeNow() / 1e6);
    robot_status_.time = msg.send_time_sec;
    navigation_debug_.update_position(
        tdttoolkit::Vec2f(robot_status_.position_x, robot_status_.position_y),
        robot_status_.yaw);
  }
  if (msg.type == 1) {
    robot_command_.goal_x = msg.position_x;
    robot_command_.goal_y = msg.position_y;
    robot_command_.next_command_id = -1;
    stop_flag = false;
    speicific_flag = tdttoolkit::NavStatus::SpecificTarget::NONE;
  }

  last_recv_time_ = msg.send_time_sec;
  // Update();
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wait_cv_.notify_one();
}
#endif
