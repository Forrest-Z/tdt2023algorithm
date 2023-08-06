#ifndef __POLYFIT_BACKEND_H__
#define __POLYFIT_BACKEND_H__
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <vector>

#include "astar_frontend.h"
#include "base_backend.h"
namespace navigation {
class PolyFit : public BaseNavigationBackend {
 public:
  struct Node {
    int x;
    int y;
    float weight = 1;
    Node(int x, int y) : x(x), y(y), weight(1){};
    Node(AStar::Node &astar_node)
        : x(astar_node.x), y(astar_node.y), weight(1){};
  };

  class Result : public BaseResult {
   private:
    Eigen::MatrixXd x_result;
    Eigen::MatrixXd y_result;

   public:
    Result(){};
    Result(Eigen::MatrixXd x, Eigen::MatrixXd y) : x_result(x), y_result(y){};

    /***********************
     * @name calcSpeed
     * @brief 计算t时刻的速度
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）
     * @return 速度向量
     */
    tdttoolkit::Vec2d calcSpeed(double t) {
      tdttoolkit::Vec2d speed;
      for (int i = 1; i < x_result.rows(); i++) {
        speed.x += i * x_result(i, 0) * pow(t, i - 1);
        speed.y += i * y_result(i, 0) * pow(t, i - 1);
      }
      return speed;
    }

    /***********************
     * @name calcPosition
     * @brief 计算t时刻的位置
     * @param [in] t 当前时间（相对于起点时间而非绝对时间）
     * @return 位置坐标
     */
    tdttoolkit::Vec2d calcPosition(double t) {
      tdttoolkit::Vec2d position;
      for (int i = 0; i < x_result.rows(); i++) {
        position.x += x_result(i, 0) * pow(t, i);
        position.y += y_result(i, 0) * pow(t, i);
      }
      return position;
    }

    void setXResult(const Eigen::MatrixXd &x) {
      if (empty_) empty_ = false;
      x_result = x;
    }
    void setYResult(const Eigen::MatrixXd &y) {
      if (empty_) empty_ = false;
      y_result = y;
    }
  };

 private:
  float calcPathAngle(std::vector<Node> &nodes, int n_per_size);

  void calcNodeWeight(std::vector<Node> &nodes);

  void calcPolyfitIndepentVariable(double indepent_variable, int n,
                                   Eigen::MatrixXd &A, int index, float weight,
                                   int derivative = 0);

 private:
  int map_width;
  int map_height;
  uchar *map;

 public:
  PolyFit();
  ~PolyFit();

  /***********************
   * @name solve
   * @brief 导航后端优化(最小二乘)
   * @param [in] nodes 前端计算的离散路径点
   * @param [in] velocity 起点速度和终端目标速度
   * @param [in] n_per_size 多少点增加一阶(已弃用)
   * @param [in] resolve_rate 每多少点取一个点进行后端优化计算
   * @param [in] front_step 前端一步为多少点
   * @param [in] time_tick_rate 每一个单位时间为多少s
   * @return 优化后的路径点
   */
  Result solve(std::vector<AStar::Node> &nodes,
               std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d> velocity,
               int n_per_size, int resolve_rate = 1, double time_tick_rate = 1);

  /***********************
   * @name solve
   * @brief 导航后端优化(最小二乘)
   * @param [in] nodes 前端计算的离散路径点
   * @param [in] velocity 起点速度和终端目标速度
   * @param [in] n_per_size 多少点增加一阶(已弃用)
   * @param [in] resolve_rate 每多少点取一个点进行后端优化计算
   * @param [in] front_step 前端一步为多少点
   * @param [in] time_tick_rate 每一个单位时间为多少s
   * @return 优化后的路径点
   */
  Result solve(std::vector<Node> &nodes,
               std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d> velocity,
               int n_per_size, int resolve_rate = 1, double time_tick_rate = 1);

  void initMap(int width, int height, uchar *map);
};
}  // namespace navigation

#endif