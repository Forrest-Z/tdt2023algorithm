#ifndef __MULTI_POLYFIT_BACKEND_H__
#define __MULTI_POLYFIT_BACKEND_H__

#include <algorithm>

#include "astar_frontend.h"
#include "base_backend.h"
#include "roborts_utils/roborts_utils.h"

namespace navigation {
class MultiPolyfitBackend : public BaseNavigationBackend {
 public:
  using Node = AStar::Node;

 public:
  class Result : public BaseResult {
   private:
    std::vector<Eigen::MatrixXd> x_forward;
    std::vector<Eigen::MatrixXd> y_forward;
    std::vector<Eigen::MatrixXd> x_backward;
    std::vector<Eigen::MatrixXd> y_backward;

    std::vector<double> time_forward;
    std::vector<double> time_backward;

   public:
    Result() {
      x_forward.clear();
      y_forward.clear();
      x_backward.clear();
      y_backward.clear();
      time_forward.clear();
      time_backward.clear();
    }

    /***********************
     * @name calcSpeed
     * @brief 计算t时刻的速度
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）单位 s
     * @return 速度向量
     */
    tdttoolkit::Vec2d calcSpeed(double t) {
      if (!time_forward.empty() && t <= time_forward.back()) {
        int id = std::lower_bound(time_forward.begin(), time_forward.end(), t) -
                 time_forward.begin();
        double delta_t = t;
        if (id > 0) {
          delta_t = t - time_forward[id - 1];
        }
        tdttoolkit::Vec2d speed;
        for (int i = 1; i < x_forward[id].rows(); i++) {
          speed.x += i * x_forward[id](i, 0) * pow(delta_t, i - 1);
          speed.y += i * y_forward[id](i, 0) * pow(delta_t, i - 1);
        }
        return speed;
      } else if (!time_backward.empty() &&
                 (t - (time_forward.empty() ? 0 : time_forward.back())) <=
                     time_backward.back()) {
        double time = t - time_forward.back();
        time = time_backward.back() - time;
        int id =
            std::lower_bound(time_backward.begin(), time_backward.end(), time) -
            time_backward.begin();
        double delta_t = time_backward[id] - time;
        tdttoolkit::Vec2d speed;
        for (int i = 1; i < x_backward[id].rows(); i++) {
          speed.x += i * x_backward[id](i, 0) * pow(delta_t, i - 1);
          speed.y += i * y_backward[id](i, 0) * pow(delta_t, i - 1);
        }
        return speed;
      } else {
        return tdttoolkit::Vec2d(0, 0);
      }
    }

    /***********************
     * @name calcPosition
     * @brief 计算t时刻的位置
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）单位 s
     * @return 位置坐标
     */
    tdttoolkit::Vec2d calcPosition(double t) {
      if (!time_forward.empty() && t <= time_forward.back()) {
        int id = std::lower_bound(time_forward.begin(), time_forward.end(), t) -
                 time_forward.begin();
        double delta_t = t;
        if (id > 0) {
          delta_t = t - time_forward[id - 1];
        }
        tdttoolkit::Vec2d position;
        for (int i = 0; i < x_forward[id].rows(); i++) {
          position.x += x_forward[id](i, 0) * pow(delta_t, i);
          position.y += y_forward[id](i, 0) * pow(delta_t, i);
        }
        return position;
      } else if (!time_backward.empty() &&
                 (t - (time_forward.empty() ? 0 : time_forward.back())) <=
                     time_backward.back()) {
        double time = t - (time_forward.empty() ? 0 : time_forward.back());
        time = time_backward.back() - time;
        int id =
            std::lower_bound(time_backward.begin(), time_backward.end(), time) -
            time_backward.begin();
        double delta_t = time_backward[id] - time;
        tdttoolkit::Vec2d position;
        for (int i = 0; i < x_backward[id].rows(); i++) {
          position.x += x_backward[id](i, 0) * pow(delta_t, i);
          position.y += y_backward[id](i, 0) * pow(delta_t, i);
        }
        return position;
      } else {
        return tdttoolkit::Vec2d(-1, -1);
      }
    }

    /***********************
     * @name addForward
     * @brief  添加后端优化的一个前向约束分段
     * @param [in] x x方向多项式系数
     * @param [in] y y方向多项式系数
     * @param [in] time 当前分段右端端点时间
     */
    void addForward(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y,
                    const double &time) {
      if (empty_) empty_ = false;
      x_forward.push_back(x);
      y_forward.push_back(y);
      time_forward.push_back(time);
    }

    /***********************
     * @name addBackward
     * @brief  添加后端优化的一个反向约束分段
     * @param [in] x x方向多项式系数
     * @param [in] y y方向多项式系数
     * @param [in] time 当前分段左端端点时间
     */
    void addBackward(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y,
                     const double &time) {
      if (empty_) empty_ = false;
      x_backward.push_back(x);
      y_backward.push_back(y);
      time_backward.push_back(time);
    }
  };

 public:
  MultiPolyfitBackend(double dis_per_pixel = 4)
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

 private:
  /***********************
   * @name calcSpeedDirection
   * @brief 计算三个点的平均速度方向
   * @param [in] start 起始点
   * @param [in] middle 中间点
   * @param [in] end 终止点
   * @return 平均速度方向
   */
  double calcSpeedDirection(const Node &start, const Node middle,
                            const Node &end);

  /***********************
   * @name calcSegmentTime
   * @brief 计算一个分段的终点速度
   * @param [in] start 起始点
   * @param [in] end 终止点
   * @param [in] acceleration 预估分段时间时所用加速度
   * @param [in] start_angle 起始点速度方向
   * @param [in] start_speed 起始点速度大小
   * @param [in] end_angle 终止点速度方向
   * @return 终止点速度大小
   */
  double calcSegmentTime(const Node &start, const Node &end,
                         const double acceleration, const double start_angle,
                         const double start_speed);
};
}  // namespace navigation

#endif