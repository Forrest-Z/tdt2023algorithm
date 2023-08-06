#ifndef __HYBIRD_ASTAR_BACKEND_H
#define __HYBIRD_ASTAR_BACKEND_H
#include <vector>

#include "astar_frontend.h"
#include "base_backend.h"
namespace navigation {
class HybridAstarBackend : BaseNavigationBackend {
  using Node = AStar::Node;

 public:
  class Result : public BaseResult {
   private:
    std::vector<Node> front_nodes;
    double astar_time_tick;

   public:
    Result() {
      LoadParam::ReadParam("navigation", "timeTickAstar", astar_time_tick);
      ;
    }

    tdttoolkit::Vec2d calcSpeed(double t) {
      int cnt_node_id = t / astar_time_tick + 1;
      cnt_node_id = 1;
      if (front_nodes.empty() || cnt_node_id > front_nodes.size() - 1) {
        return tdttoolkit::Vec2d(0, 0);
      }
      return tdttoolkit::Vec2d(front_nodes[cnt_node_id].speed_x,
                               front_nodes[cnt_node_id].speed_y);
    }

    tdttoolkit::Vec2d calcPosition(double t) {
      int cnt_node_id = t / astar_time_tick;
      if (cnt_node_id > front_nodes.size() - 1) {
        return tdttoolkit::Vec2d(-1, -1);
      }
      auto time = t - cnt_node_id * astar_time_tick;
      return tdttoolkit::Vec2d(front_nodes[cnt_node_id].x +
                                   time * front_nodes[cnt_node_id + 1].speed_x,
                               front_nodes[cnt_node_id].y +
                                   time * front_nodes[cnt_node_id + 1].speed_y);
    }

    void setNodes(std::vector<Node> &nodes) {
      this->front_nodes = nodes;
      this->empty_ = false;
    }
  };

  Result solve(std::vector<Node> &nodes,  // 前端路径点
               std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d>
                   velocity,      // 起始点和终点速度约束(未使用)
               int order,         // 多项式阶数(未使用)
               int resolve_rate,  // 分段解析度(未使用)
               double acceleration,  // 预估分段时间时所用加速度(未使用)
               double max_velocity,  // 全局最大速度(未使用)
               bool fixed_start      // 是否修正了起点(未使用)
  );

 private:
};
}  // namespace navigation

#endif