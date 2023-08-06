#include "controler.h"

#include <roborts_utils/base_toolkit.h>

#include <rclcpp/qos.hpp>

namespace navigation {
Controler::Controler(double dis_per_pixel, NavigationDebug *debug) {
  dis_per_pixel_ = dis_per_pixel;
  ros_node_ = ros_node_ =
      std::make_shared<rclcpp::Node>("navigation_controler");
  LoadParam::ReadParam("navigation", "max_turn_speed", max_turn_speed);
  LoadParam::ReadParam("navigation", "max_acc", max_acc);
  max_acc /= dis_per_pixel;
  debug_ = debug;
}

Controler::Controler(const Controler &other) {
  result_ = other.result_;
  start_time_ = other.start_time_;
  robot_status_ = other.robot_status_;
  // task_schuduler_ = other.task_schuduler_;
  ros_node_ = other.ros_node_;
  usart_publisher = other.usart_publisher;
  control_fps_ = other.control_fps_;
  dis_per_pixel_ = other.dis_per_pixel_;
  regen_time_ = other.regen_time_;
  // mutex_ = other.mutex_;
  stop_ = other.stop_;
}

void Controler::set_result(const Result &result, const double &start_time) {
  // mutex_.lock();
  result_ = result;
  start_time_ = start_time;
  stop_ = false;
  // mutex_.unlock();
}

void Controler::set_robot_pos(const double &x, const double &y,
                              const double &time_stamp) {
  // mutex_.lock();
  if (time_stamp < robot_status_.time_stamp) {
    return;
  }
  robot_status_.pos_x = x;
  robot_status_.pos_y = y;
  robot_status_.time_stamp = time_stamp;
  // mutex_.unlock();
}

void Controler::set_robot_speed(const double &x, const double &y,
                                const double &time_stamp) {
  // mutex_.lock();
  if (time_stamp < robot_status_.time_stamp) {
    return;
  }
  robot_status_.pos_x +=
      robot_status_.speed_x * (time_stamp - robot_status_.time_stamp);
  robot_status_.pos_y +=
      robot_status_.speed_y * (time_stamp - robot_status_.time_stamp);
  robot_status_.speed_x = x;
  robot_status_.speed_y = y;
  robot_status_.time_stamp = time_stamp;
  // mutex_.unlock();
}

void Controler::set_robot_euler(const double &yaw, const double &pitch,
                                const double &roll, const double &time_stamp) {
  if (time_stamp < robot_euler_.timestamp) {
    return;
  }
  // mutex_.lock();
  robot_euler_.yaw = yaw;
  robot_euler_.pitch = pitch;
  robot_euler_.roll = roll;
  robot_euler_.timestamp = time_stamp;
  // mutex_.unlock();
}

void Controler::set_stop() {
  // mutex_.lock();
  stop_ = true;
  // mutex_.unlock();
}

tdttoolkit::Vec2d Controler::GetOutputSpeed() { return last_speed; }

void Controler::init() {
  usart_publisher =
      ros_node_->create_publisher<navigation_interface::msg::Navigation2Usart>(
          "navigation2usart", rclcpp::SensorDataQoS());
}

void Controler::start() {
  task_schuduler_.async_run(1000.0 / control_fps_, &Controler::update, this);
}

void Controler::stop() { task_schuduler_.async_stop(); }

void Controler::update() {
  navigation_interface::msg::Navigation2Usart msg;
  auto cnt_time = tdttoolkit::Time::GetTimeNow() / 1e6;
  // mutex_.lock();
  msg.lifting = false;
  if (target_type == (int)'A' || target_type == (int)'Z') {
    if (on_arrive) {
      msg.lifting = true;
    }
  }

  if (target_type == 'D') {
    msg.spin = false;
  } else {
    msg.spin = true;
  }

  if (cnt_time - robot_status_.time_stamp > 1.5 ||
      cnt_time - start_time_ > 1.5 || stop_) {
    if (last_speed.length() < max_acc / control_fps_) {
      last_speed = tdttoolkit::Vec2d(0, 0);
    } else {
      last_speed -= last_speed.normalized() * max_acc / control_fps_;
    }
    msg.time = cnt_time;
    auto supposed_speed_size = last_speed.length();
    auto supposed_speed_towords = last_speed.angle();
    msg.x_speed = supposed_speed_size *
                  cos(supposed_speed_towords + robot_euler_.yaw) *
                  dis_per_pixel_ * 10;  // 转化为 mm/s
    msg.y_speed = supposed_speed_size *
                  sin(supposed_speed_towords + robot_euler_.yaw) *
                  dis_per_pixel_ * 10;

    usart_publisher->publish(msg);

    // mutex_.unlock();
    return;
  }
  auto time_diff = cnt_time - start_time_;
  auto cnt_pos_x =
      robot_status_.pos_x +
      robot_status_.speed_x * (cnt_time - robot_status_.time_stamp);
  auto cnt_pos_y =
      robot_status_.pos_y +
      robot_status_.speed_y * (cnt_time - robot_status_.time_stamp);
  // auto supposed_position = result_.calcPosition(time_diff);
  auto supposed_speed = result_.calcSpeed(time_diff);
  // TDT_INFO("orgin speed size:%f, angle=%f", supposed_speed.length(),
  //          supposed_speed.angle());
  last_speed = supposed_speed;
  auto supposed_speed_size = supposed_speed.length();
  auto supposed_speed_towords = supposed_speed.angle();
  // static double last_speed_towords = supposed_speed_towords;
  // if (fabs(tdttoolkit::AngleLimit(supposed_speed_towords -
  //                                 last_speed_towords)) *
  //         control_fps_ >
  //     max_turn_speed) {
  //   supposed_speed_towords =
  //       supposed_speed_towords > last_speed_towords
  //           ? last_speed_towords + max_turn_speed / control_fps_
  //           : last_speed_towords - max_turn_speed * control_fps_;
  // }
  // last_speed_towords = supposed_speed_towords;

  msg.time = cnt_time;
  msg.x_speed = supposed_speed_size *
                cos(supposed_speed_towords + robot_euler_.yaw) *
                dis_per_pixel_ * 10;  // 转化为 mm/s
  msg.y_speed = supposed_speed_size *
                sin(supposed_speed_towords + robot_euler_.yaw) *
                dis_per_pixel_ * 10;

  usart_publisher->publish(msg);
  // mutex_.unlock();
}

}  // namespace navigation