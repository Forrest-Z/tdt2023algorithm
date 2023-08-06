#ifndef VISION_DEBUG_H
#define VISION_DEBUG_H

#include <yaml-cpp/yaml.h>

#include <vector>

#include "base_interface/msg/color.hpp"
#include "base_interface/msg/display_param.hpp"
#include "base_interface/msg/param.hpp"
#include "base_interface/msg/point2d.hpp"
#include "base_interface/msg/point2f.hpp"
#include "base_interface/msg/rect.hpp"
#include "opencv2/opencv.hpp"
#include "vision_interface/msg/draw_circle.hpp"
#include "vision_interface/msg/draw_line.hpp"
#include "vision_interface/msg/draw_rect.hpp"
#include "vision_interface/msg/draw_text.hpp"
#include "vision_interface/msg/group_msgs.hpp"
#include "vision_interface/srv/request_vision_param.hpp"
#include "vision_interface/srv/apply_vision_param.hpp"



// #include "parameter/load_param.h"
#include "base_param.h"
// #include "rclcpp_action/rclcpp_action.hpp"
#include "string.h"
namespace tdttoolkit {
class Debug {
 public:
  Debug() {}

  static inline void drawRect(vision_interface::msg::GroupMsgs& msg,
                              std::string group_name,
                              std::vector<cv::Point2f> points,
                              int thickness = 1,
                              cv::Scalar color = cv::Scalar(255, 255, 255)) {
    auto rect = vision_interface::msg::DrawRect();

    std::vector<base_interface::msg::Point2f> rect_points;

    for (auto& point : points) {
      auto point_ = base_interface::msg::Point2f();
      point_.x = point.x;
      point_.y = point.y;
      rect_points.push_back(point_);
    }

    rect.rect.lt = rect_points.at(0);
    rect.rect.lb = rect_points.at(1);
    rect.rect.rt = rect_points.at(2);
    rect.rect.rb = rect_points.at(3);

    rect.thickness = thickness;

    auto colcor_ = base_interface::msg::Color();
    colcor_.r = color[0];
    colcor_.g = color[1];
    colcor_.b = color[2];

    rect.color = colcor_;

    msg.rects.push_back(rect);
  }

  static inline void drawPoint(vision_interface::msg::GroupMsgs& msg,
                               std::string group_name, cv::Point2f point2f,
                               int radius = 1, int thickness = 1,
                               cv::Scalar color = cv::Scalar(255, 255, 255)) {
    auto circle = vision_interface::msg::DrawCircle();
    auto point = base_interface::msg::Point2f();

    point.x = point2f.x;
    point.y = point2f.y;

    auto colcor_ = base_interface::msg::Color();
    colcor_.r = color[0];
    colcor_.g = color[1];
    colcor_.b = color[2];

    circle.group_name = group_name;
    circle.center = point;
    circle.radius = radius;
    circle.thickness = thickness;
    circle.color = colcor_;

    msg.circles.push_back(circle);
  }

  static inline void drawTxt(vision_interface::msg::GroupMsgs& msg,
                             std::string group_name, cv::Point2f position,
                             std::string txt, int font_size, int thickness = 1,
                             cv::Scalar color = cv::Scalar(255, 255, 255)) {
    auto text = vision_interface::msg::DrawText();
    auto point = base_interface::msg::Point2f();

    point.x = position.x;
    point.y = position.y;

    auto colcor_ = base_interface::msg::Color();
    colcor_.r = color[0];
    colcor_.g = color[1];
    colcor_.b = color[2];

    text.group_name = group_name;
    text.position = point;
    text.text = txt;
    text.font_size = font_size;
    text.thickness = thickness;
    text.color = colcor_;

    msg.texts.push_back(text);
  }

  static inline void DisplayParam(vision_interface::msg::GroupMsgs& msg,
                                  std::string group_name,
                                  std::string param_name,
                                  std::string param_value) {
    auto param = base_interface::msg::DisplayParam();
    param.group_name = group_name;
    param.param_name = param_name;
    param.param_value = param_value;
    msg.params.push_back(param);
  }

  static inline base_interface::msg::Color Color(int r, int g, int b) {
    auto color = base_interface::msg::Color();
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  static inline base_interface::msg::Point2f Point2f(float x, float y) {
    auto point = base_interface::msg::Point2f();
    point.x = x;
    point.y = y;
    return point;
  }

  static inline base_interface::msg::Point2d Point2d(int x, int y) {
    auto point = base_interface::msg::Point2d();
    point.x = x;
    point.y = y;
    return point;
  }

  static inline base_interface::msg::Rect Rect(
      base_interface::msg::Point2f lt, base_interface::msg::Point2f lb,
      base_interface::msg::Point2f rt, base_interface::msg::Point2f rb) {
    auto rect = base_interface::msg::Rect();
    rect.lt = lt;
    rect.lb = lb;
    rect.rt = rt;
    rect.rb = rb;
    return rect;
  }
};

class Debug_param {
 public:
  Debug_param() {}

  static  void sendParamList(std::string path,
      const vision_interface::srv::RequestVisionParam::Response::SharedPtr response) {
    
    YAML::Node config  =YAML::LoadFile(path); 
    auto parameter_dynamic = config["parameter_dynamic"];

    for (const auto& i : parameter_dynamic) {
      // std::cout << i.first.as<std::string>() << std::endl;
      auto group_name = i["group_name"].as<std::string>();

      for (const auto& j : i["item"]) {
        auto param = base_interface::msg::Param();

        auto item_name = j["item_name"].as<std::string>();

        auto item_type = j["item_type"].as<std::string>();

        param.group_name = group_name;
        param.item.name = item_name;
        param.item.type = item_type;

        if (item_type == "float") {
          auto item_reference = j["item_reference"].as<float>();
          auto item_min = j["item_min"].as<float>();
          auto item_max = j["item_max"].as<float>();

          param.item.reference = std::to_string(item_reference);
          param.item.min = std::to_string(item_min);
          param.item.max = std::to_string(item_max);
        }

        else if (item_type == "int") {
          auto item_reference = j["item_reference"].as<int>();
          auto item_min = j["item_min"].as<int>();
          auto item_max = j["item_max"].as<int>();

          param.item.reference = std::to_string(item_reference);
          param.item.min = std::to_string(item_min);
          param.item.max = std::to_string(item_max);
        }

        else if (item_type == "bool") {
          auto item_reference = j["item_reference"].as<bool>();
          auto item_min = j["item_min"].as<bool>();
          auto item_max = j["item_max"].as<bool>();

          param.item.reference = std::to_string(item_reference);
          param.item.min = std::to_string(item_min);
          param.item.max = std::to_string(item_max);
        }

        // 添加信息选择发送

        response->params_dynamic.push_back(param);
      }
    }

      auto parameter_static = config["parameter_static"];
      for (const auto& i : parameter_static){
        auto group_name = i["group_name"].as<std::string>();

      for (const auto& j : i["item"]){
        auto param = base_interface::msg::Param();

        auto item_name = j["item_name"].as<std::string>();

        auto item_type = j["item_type"].as<std::string>();

        param.group_name = group_name;
        param.item.name = item_name;
        param.item.type = item_type;

        response->params_static.push_back(param);
      } 

      }
  }

 private:
};
// namespace Debug
}  // namespace tdttoolkit
#endif