#include "hybrid_astar_backend.h"

namespace navigation {
HybridAstarBackend::Result HybridAstarBackend::solve(
    std::vector<Node> &nodes,  // 前端路径点
    std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d>
        velocity,         // 起始点和终点速度约束(未使用)
    int order,            // 多项式阶数(未使用)
    int resolve_rate,     // 分段解析度(未使用)
    double acceleration,  // 预估分段时间时所用加速度(未使用)
    double max_velocity,  // 全局最大速度(未使用)
    bool fixed_start      // 是否修正了起点(未使用)
) {
  Result result;
  result.setNodes(nodes);
  return result;
}
}  // namespace navigation