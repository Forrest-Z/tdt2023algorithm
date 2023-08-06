#ifndef __DIRECT_BACKEND_H
#define __DIRECT_BACKEND_H
#include <roborts_utils/base_param.h>

#include "astar_frontend.h"
#include "base_backend.h"
namespace navigation {
class DirectBackend : BaseNavigationBackend {
  using Node = AStar::Node;

 public:
  class Result : public BaseResult {
   private:
    tdttoolkit::Vec2d speed;
    float direct_backend_toword_filter_rate, direct_backend_speed_filter_rate,
        direct_backend_speed_down_toword_thre, direct_backend_speed_down_rate;

   public:
    Result() {
      LoadParam::ReadParam("navigation", "direct_backend_toword_filter_rate",
                           direct_backend_toword_filter_rate);
      LoadParam::ReadParam("navigation", "direct_backend_speed_filter_rate",
                           direct_backend_speed_filter_rate);
      LoadParam::ReadParam("navigation",
                           "direct_backend_speed_down_toword_thre",
                           direct_backend_speed_down_toword_thre);
      LoadParam::ReadParam("navigation", "direct_backend_speed_down_rate",
                           direct_backend_speed_down_rate);
    };

    tdttoolkit::Vec2d calcSpeed(double t) {
      static double last_speed_size = speed.length();
      static double last_speed_toword = speed.angle();
      // TDT_INFO("orgin speed1:%f",speed.length());
      auto cnt_speed_size =
          last_speed_size * (1 - direct_backend_speed_filter_rate) +
          speed.length() * direct_backend_speed_filter_rate;
      // TDT_INFO("orgin last speed:%f",last_speed_size);
      // TDT_INFO("orgin
      // direct_backend_speed_filter_rate:%f",direct_backend_speed_filter_rate);
      // TDT_INFO("orgin speed2:%f",cnt_speed_size);
      auto cnt_speed_toword =
          last_speed_toword * (1 - direct_backend_toword_filter_rate) +
          speed.angle() * direct_backend_toword_filter_rate;
      last_speed_size = cnt_speed_size;
      last_speed_toword = cnt_speed_toword;
      if (cnt_speed_toword > direct_backend_speed_down_toword_thre)
        cnt_speed_size *= direct_backend_speed_down_rate;

      auto ret_speed =
          tdttoolkit::Vec2d::polor(cnt_speed_size, cnt_speed_toword);
      //  TDT_INFO("orgin speed3:%f",ret_speed.length());
      return ret_speed;
    }

    tdttoolkit::Vec2d calcPosition(double t) {
      return tdttoolkit::Vec2d(-1, -1);
    }

    void setSpeed(tdttoolkit::Vec2d speed) { this->speed = speed; }
  };

  Result solve(std::vector<Node> &nodes,  // 前端路径点
               std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d>
                   velocity,      // 起始点和终点速度约束(未使用)
               int order,         // 多项式阶数(未使用)
               int resolve_rate,  // 分段解析度(未使用)
               double acceleration,  // 预估分段时间时所用加速度(未使用)
               double max_velocity,  // 全局最大速度(未使用)
               bool fixed_start      // 是否修正了起点(未使用)
  ) {
    Result result;
    tdttoolkit::Vec2d speed;
    int target_pos = lead_size < nodes.size() ? lead_size : nodes.size() - 1;
    speed.x = (nodes[target_pos].x - nodes[0].x);
    speed.y = (nodes[target_pos].y - nodes[0].y);
    auto speed_size = max_velocity;
    if ((!fixed_start) && nodes.size() < speed_down_size)
      speed_size = max_velocity *
                   (nodes.size() - 1 < 0 ? 0 : nodes.size() - 1) /
                   speed_down_size;
    speed = speed.normalized() * speed_size;
    result.setSpeed(speed);
    return result;
  };

 private:
  int lead_size = 4;
  int speed_down_size = 6;
};
}  // namespace navigation

#endif