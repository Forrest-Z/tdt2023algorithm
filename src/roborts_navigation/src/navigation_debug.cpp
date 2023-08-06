#include "navigation_debug.h"

#include <cv_bridge/cv_bridge.h>
namespace navigation {
NavigationDebug::NavigationDebug() {
  LoadParam::ReadParam("navigation_map", "dis_per_pixel", dis_per_pixel_);
  LoadParam::ReadParam("navigation", "perception_info_time_out",
                       robot_info_time_out);
  LoadParam::ReadParam("navigation", "debug_scale_rate", debug_scale_rate_);
  LoadParam::ReadParam("navigation", "self_radius", radius);
  LoadParam::ReadParam("navigation", "enemy_radius", enemy_radius);
  radius /= dis_per_pixel_;
}

void NavigationDebug::init() {
  ros_node_ = rclcpp::Node::make_shared("navigation_debug");
  debug_publisher_ =
      ros_node_->create_publisher<navigation_interface::msg::NavigationDebug>(
          "navigation_debug", 10);
  std::string map_name;
  LoadParam::ReadParam("navigation", "map_name", map_name);
  show_map_ = cv::imread(
      "install/roborts_navigation/share/roborts_navigation/show_map/" +
      map_name + ".png");
  map_hash_ = tdttoolkit::MatHash(show_map_);
  debug_map_service =
      ros_node_->create_service<navigation_interface::srv::Nav2DebugMap>(
          "NavMap2Debug",
          std::bind(&NavigationDebug::OnReqNavMap, this, std::placeholders::_1,
                    std::placeholders::_2));
  qtdebug_publisher_ =
      ros_node_->create_publisher<navigation_interface::msg::Navigation2Debug>(
          "navigation2_debug", rclcpp::SensorDataQoS());
  debug2nav_subscriber_ =
      ros_node_->create_subscription<navigation_interface::msg::Debug2Nav>(
          "debug2_navigation", 10,
          std::bind(&NavigationDebug::OnDebug2Nav, this,
                    std::placeholders::_1));
}

void NavigationDebug::OnMouse(int event, int x, int y, int flags, void *param) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    ((NavigationDebug *)param)->position.x =
        x / ((NavigationDebug *)param)->debug_scale_rate_;
    ((NavigationDebug *)param)->position.y =
        y / ((NavigationDebug *)param)->debug_scale_rate_;
    double time = tdttoolkit::Time::GetTimeNow();
    navigation_interface::msg::NavigationDebug msg;
    msg.type = 0;
    msg.position_x = ((NavigationDebug *)param)->position.x;
    msg.position_y = ((NavigationDebug *)param)->position.y;
    msg.send_time_sec = time / 1e6;
    ((NavigationDebug *)param)->debug_publisher_->publish(msg);
  } else if (event == cv::EVENT_MBUTTONDOWN) {
    ((NavigationDebug *)param)->target.x =
        x / ((NavigationDebug *)param)->debug_scale_rate_;
    ((NavigationDebug *)param)->target.y =
        y / ((NavigationDebug *)param)->debug_scale_rate_;
    double time = tdttoolkit::Time::GetTimeNow();
    navigation_interface::msg::NavigationDebug msg;
    msg.type = 1;
    msg.position_x = ((NavigationDebug *)param)->target.x;
    msg.position_y = ((NavigationDebug *)param)->target.y;
    msg.send_time_sec = time / 1e6;
    ((NavigationDebug *)param)->debug_publisher_->publish(msg);
  }
}

void NavigationDebug::OnDebug2Nav(
    const navigation_interface::msg::Debug2Nav::SharedPtr msg) {
  if (msg->type == 0) {
    position.x = msg->point.x;
    position.y = msg->point.y;
    double time = tdttoolkit::Time::GetTimeNow();
    navigation_interface::msg::NavigationDebug new_msg;
    new_msg.type = 0;
    new_msg.position_x = position.x;
    new_msg.position_y = position.y;
    new_msg.send_time_sec = time / 1e6;
    debug_publisher_->publish(new_msg);
  } else if (msg->type == 1) {
    target.x = msg->point.x;
    target.y = msg->point.y;
    double time = tdttoolkit::Time::GetTimeNow();
    navigation_interface::msg::NavigationDebug new_msg;
    new_msg.type = 1;
    new_msg.position_x = target.x;
    new_msg.position_y = target.y;
    new_msg.send_time_sec = time / 1e6;
    debug_publisher_->publish(new_msg);
  }
}

void NavigationDebug::add_param(const std::string &name, int &value,
                                const int max_value) {
  params_[name] = &value;
}

void NavigationDebug::workthread() {
  cv::namedWindow("navigation_debug");
  cv::setMouseCallback("navigation_debug", OnMouse, this);
  for (auto &param : params_)
    cv::createTrackbar(param.first, "navigation_debug", param.second, 50, NULL);
  while (running_) {
    cv::Mat show_map = show_map_.clone();
    cv::resize(show_map, show_map,
               cv::Size(show_map.cols * debug_scale_rate_,
                        show_map.rows * debug_scale_rate_));
    if (position.x > 0 && position.y > 0)
      cv::circle(show_map,
                 cv::Point(position.x * debug_scale_rate_,
                           position.y * debug_scale_rate_),
                 5, cv::Scalar(0, 0, 255), -1);
    if (target.x > 0 && target.y > 0)
      cv::circle(
          show_map,
          cv::Point(target.x * debug_scale_rate_, target.y * debug_scale_rate_),
          5, cv::Scalar(0, 255, 0), -1);

    if (frontnodes.size() > 0) {
      for (auto &node : frontnodes) {
        cv::circle(
            show_map,
            cv::Point(node.x * debug_scale_rate_, node.y * debug_scale_rate_),
            1, cv::Scalar(255, 0, 0), -1);
      };
    }
    if (!backendresult.isEmpty())
      for (double t = 0;; t += 0.01) {
        auto pos = backendresult.calcPosition(t);
        if (pos.x < 0 || pos.y < 0 || pos.x > show_map.cols ||
            pos.y > show_map.rows || t > 10000)
          break;
        cv::circle(
            show_map,
            cv::Point(pos.x * debug_scale_rate_, pos.y * debug_scale_rate_), 2,
            cv::Scalar(0, 0, 255), 1);
      }
    for (int i = 0; i < 2 * army_type_num; i++)
      ;
    cv::imshow("navigation_debug", show_map);
    cv::waitKey(10);
  }
}

void NavigationDebug::start() {
  running_ = true;
  LoadParam::ReadParam("navigation", "video_debug_local", running_);
  if (running_) {
    std::thread workthread(&NavigationDebug::workthread, this);
    workthread.detach();
  }
  std::thread([this]() {
    rclcpp::spin(ros_node_);
    rclcpp::shutdown();
  }).detach();
}

void NavigationDebug::stop() { running_ = false; }

void NavigationDebug::OnReqNavMap(
    const std::shared_ptr<navigation_interface::srv::Nav2DebugMap::Request> req,
    std::shared_ptr<navigation_interface::srv::Nav2DebugMap::Response> resp) {
  if (req->require_map) {
    resp->header.frame_id = "Init";
    resp->header.stamp = tdttoolkit::Time::GetROSTimeNow();
    resp->image =
        *cv_bridge::CvImage(resp->header, "bgr8", show_map_).toImageMsg();
    resp->map_hash = map_hash_;
  }
}

void NavigationDebug::publish_frame(double tick_time) {
  navigation_interface::msg::Navigation2Debug msg;
  msg.header.frame_id = "";
  msg.header.stamp = tdttoolkit::Time::GetRosTimeByTime(tick_time);
  msg.map_hash = map_hash_;
  if (position.x > 0 && position.y > 0) {
    navigation_interface::msg::NavDrawCircle circles;
    circles.color = tdttoolkit::Debug::Color(205, 50, 205);
    circles.radius = radius;
    circles.thickness = 1;
    circles.centers.push_back(
        tdttoolkit::Debug::Point2d(position.x, position.y));
    msg.circles.push_back(circles);

    // interface::msg::NavDrawText texts;
    // texts.text = "起点";
    // texts.color = tdttoolkit::Debug::Color(205, 50, 205);
    // texts.positions.push_back(
    //     tdttoolkit::Debug::Point2d(position.x, position.y));
    // texts.font_size = 2;
    // texts.thickness = 1;
    // msg.texts.push_back(texts);

    navigation_interface::msg::NavDrawIcon icons;
    icons.icon_id = 0;
    icons.color = tdttoolkit::Debug::Color(205, 50, 205);
    icons.positions.push_back(
        tdttoolkit::Debug::Point2d(position.x, position.y));
    icons.width = 10;
    icons.height = 10;
    icons.angle = 180 + yaw / CV_PI * 180;
    msg.icons.push_back(icons);
  }
  if (target.x > 0 && target.y > 0) {
    navigation_interface::msg::NavDrawCircle circles;
    circles.color = tdttoolkit::Debug::Color(255, 128, 128);
    circles.radius = radius;
    circles.thickness = 1;
    circles.centers.push_back(tdttoolkit::Debug::Point2d(target.x, target.y));
    msg.circles.push_back(circles);

    // interface::msg::NavDrawText texts;
    // texts.text = "终点";
    // texts.color = tdttoolkit::Debug::Color(255, 128, 128);
    // texts.positions.push_back(tdttoolkit::Debug::Point2d(target.x,
    // target.y)); texts.font_size = 2; texts.thickness = 1;
    // msg.texts.push_back(texts);

    navigation_interface::msg::NavDrawIcon icons;
    icons.icon_id = 1;
    icons.color = tdttoolkit::Debug::Color(255, 128, 128);
    icons.positions.push_back(tdttoolkit::Debug::Point2d(target.x, target.y));
    icons.width = 10;
    icons.height = 10;
    icons.angle = 0;
    msg.icons.push_back(icons);
  }
  if (frontnodes.size() > 0 && front_update_time == tick_time) {
    navigation_interface::msg::NavDrawCircle circles;
    circles.color = tdttoolkit::Debug::Color(230, 89, 166);
    circles.radius = 1;
    circles.thickness = 1;
    for (auto &node : frontnodes) {
      circles.centers.push_back(tdttoolkit::Debug::Point2d(node.x, node.y));
    };
    msg.circles.push_back(circles);
  }
  if (!backendresult.isEmpty() && backend_update_time == tick_time) {
    navigation_interface::msg::NavDrawLine lines;
    lines.start_color = tdttoolkit::Debug::Color(205, 50, 205);
    lines.end_color = tdttoolkit::Debug::Color(255, 128, 128);
    lines.thickness = 1;
    for (double t = 0;; t += 0.05) {
      auto pos = backendresult.calcPosition(t);
      if (pos.x < 0 || pos.y < 0 || pos.x > show_map_.cols ||
          pos.y > show_map_.rows || t > 100)
        break;
      lines.joint_point.push_back(tdttoolkit::Debug::Point2d(pos.x, pos.y));
    }
    msg.lines.push_back(lines);
  }
  for (int i = 0; i < 2 * army_type_num; i++) {
    if (tick_time - robot_info[i].recv_time < robot_info_time_out &&
        robot_info[i].recv_time > 0) {
      navigation_interface::msg::NavDrawCircle circles;
      if (i >= army_type_num) {
        circles.color = tdttoolkit::Debug::Color(255, 51, 51);
      } else {
        circles.color = tdttoolkit::Debug::Color(51, 153, 255);
      }
      circles.radius = enemy_radius / dis_per_pixel_;
      circles.thickness = 1;
      circles.centers.push_back(
          tdttoolkit::Debug::Point2d(robot_info[i].x, robot_info[i].y));
      msg.circles.push_back(circles);

      navigation_interface::msg::NavDrawText texts;
      texts.text = '0' + i % army_type_num + 1;
      if (i >= army_type_num) {
        texts.color = tdttoolkit::Debug::Color(255, 51, 51);
      } else {
        texts.color = tdttoolkit::Debug::Color(51, 153, 255);
      }
      texts.positions.push_back(
          tdttoolkit::Debug::Point2d(robot_info[i].x, robot_info[i].y));
      texts.font_size = 4;
      texts.thickness = 1;
      msg.texts.push_back(texts);
    }
  }
  qtdebug_publisher_->publish(msg);
}

}  // namespace navigation