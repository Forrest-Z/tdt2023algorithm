#include "minimumsnap.h"

#include <OsqpEigen/OsqpEigen.h>
#include <osqp.h>
namespace navigation {
MinimumSnapBackend::Result MinimumSnapBackend::solve(
    std::vector<Node> &nodes,  // 前端路径点
    std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d>
        velocity,      // 起始点和终点速度约束
    int order,         // 多项式阶数
    int resolve_rate,  // 分段解析度(每段包含多少点)
    double acceleration,  // 预估分段时间时所用加速度(应小于实际最大加速度)
    double max_velocity  // 全局最大速度(无法约束到具体点)
) {
  ;
};

}  // namespace navigation