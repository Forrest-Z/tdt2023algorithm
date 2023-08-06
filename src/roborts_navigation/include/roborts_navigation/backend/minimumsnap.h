#ifndef __POLYFIT_BACKEND_H__
#define __POLYFIT_BACKEND_H__

#include <algorithm>

#include "astar_frontend.h"
#include "base_backend.h"
#include "roborts_utils/roborts_utils.h"

namespace navigation {
class MinimumSnapBackend : public BaseNavigationBackend {
 public:
  using Node = AStar::Node;

 public:
  class Result : public BaseResult {
   private:
    ;
    ;

   public:
    Result() {
      ;
      ;
    }

    /***********************
     * @name calcSpeed
     * @brief 计算t时刻的速度
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）单位 s
     * @return 速度向量
     */
    tdttoolkit::Vec2d calcSpeed(double t) {
      ;
      ;
    }

    /***********************
     * @name calcPosition
     * @brief 计算t时刻的位置
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）单位 s
     * @return 位置坐标
     */
    tdttoolkit::Vec2d calcPosition(double t) {
      ;
      ;
    }
  };

 public:
  MinimumSnapBackend(double dis_per_pixel = 4)
      : dis_per_pixel_(dis_per_pixel){};

  /***********************
   * @name solve
   * @brief 分段多项式优化
   * @param [in] nodes 前端路径点
   * @param [in] velocity 起始点和终点速度约束
   * @param [in] order 多项式阶数
   * @param [in] resolve_rate 分段解析度(每段包含多少点)
   * @param [in] acceleration
   * 预估分段时间时所用加速度(应小于实际最大加速度)    单位: pixel/(s^2)
   * @param [in] max_velocity 全局最大速度(无法约束到具体点)    单位: pixel/s
   * @return 优化结果
   */
  Result solve(std::vector<Node> &nodes,
               std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d> velocity,
               int order = 5, int resolve_rate = 5, double acceleration = 200,
               double max_velocity = 300);

 private:
  double dis_per_pixel_ = 4;  // 单位cm/pixel
};
}  // namespace navigation

#endif