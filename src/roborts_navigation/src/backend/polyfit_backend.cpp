#include "polyfit_backend.h"

namespace navigation {
PolyFit::PolyFit() {}

PolyFit::~PolyFit() {}

PolyFit::Result PolyFit::solve(
    std::vector<Node> &nodes,
    std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d> velocity, int n_per_size,
    int resolve_rate, double time_tick_rate) {
  // float angle = calcPathAngle(nodes, n_per_size);
  calcNodeWeight(nodes);
  // int             n          = std::log((angle - 3) > 0 ? angle + 1 : 1) + 2;
  //   int n = 7;
  int n = n_per_size;
  int point_size = nodes.size() / resolve_rate;
  float acc_weight = 0.0;
  Eigen::MatrixXd A_x(2 * point_size + 2, n);
  Eigen::MatrixXd b_x(2 * point_size + 2, 1);
  Eigen::MatrixXd A_y(2 * point_size + 2, n);
  Eigen::MatrixXd b_y(2 * point_size + 2, 1);
  for (int i = 0; i < point_size; i++) {
    int weight = nodes[i * resolve_rate].weight;
    if (i == 0 || i == point_size - 1) {
      weight = 100 * point_size;
    }
    calcPolyfitIndepentVariable(i * time_tick_rate, n, A_x, i, weight);
    calcPolyfitIndepentVariable(i * time_tick_rate, n, A_y, i, weight);
    calcPolyfitIndepentVariable(i * time_tick_rate, n, A_x, point_size + i,
                                acc_weight, 2);
    calcPolyfitIndepentVariable(i * time_tick_rate, n, A_y, point_size + i,
                                acc_weight, 2);
    b_x(i, 0) = nodes[i * resolve_rate].x * weight;
    b_y(i, 0) = nodes[i * resolve_rate].y * weight;
    b_x(point_size + i, 0) = 0;
    b_y(point_size + i, 0) = 0;
  }
  int weight = 1000 * point_size;
  b_x(2 * point_size, 0) = velocity.first.x * weight;
  b_y(2 * point_size, 0) = velocity.first.y * weight;
  b_x(2 * point_size + 1, 0) = velocity.second.x * weight;
  b_y(2 * point_size + 1, 0) = velocity.second.y * weight;
  calcPolyfitIndepentVariable(0, n, A_x, 2 * point_size, weight, 1);
  calcPolyfitIndepentVariable(0, n, A_y, 2 * point_size, weight, 1);
  calcPolyfitIndepentVariable((point_size - 1) * time_tick_rate, n, A_x,
                              2 * point_size + 1, weight, 1);
  calcPolyfitIndepentVariable((point_size - 1) * time_tick_rate, n, A_y,
                              2 * point_size + 1, weight, 1);
  Eigen::MatrixXd x_x = A_x.colPivHouseholderQr().solve(b_x);
  Eigen::MatrixXd x_y = A_y.colPivHouseholderQr().solve(b_y);
  Result result;
  result.setXResult(x_x);
  result.setYResult(x_y);
  return result;
}

PolyFit::Result PolyFit::solve(
    std::vector<AStar::Node> &nodes,
    std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d> velocity, int n_per_size,
    int resolve_rate, double time_tick_rate) {
  std::vector<Node> new_nodes;
  for (auto &node : nodes) {
    new_nodes.push_back(node);
  }
  return solve(new_nodes, velocity, n_per_size, resolve_rate, time_tick_rate);
}

float PolyFit::calcPathAngle(std::vector<Node> &nodes, int n_per_size) {
  float all_angle = 0;
  int size = nodes.size() / n_per_size;
  for (int i = 0; i < n_per_size - 1; i++) {
    float angle = 0;
    for (int j = i * size; j < (i + 1) * size; j++) {
      angle += atan2(nodes[j + 1].x - nodes[j].x, nodes[j + 1].y - nodes[j].y);
    }
    all_angle += fabs(angle);
  }
  return all_angle;
}

void PolyFit::calcNodeWeight(std::vector<Node> &nodes) {
  for (auto &node : nodes) {
    int dis = map[node.x + node.y * map_width];
    node.weight = 100.0 / (dis * dis);
    if (node.weight < 1) {
      node.weight = 1;
    }
  }
  int judge_dis = 1;
  float last_angle = 0;
  int nums = 0;
  int last_joint = -1;
  int last_nums = 0;
  for (int i = 0; i < nodes.size() - judge_dis; i++) {
    float angle = atan2(nodes[i + judge_dis].x - nodes[i].x,
                        nodes[i + judge_dis].y - nodes[i].y);
    if (i > 0 && fabs(angle - last_angle) > 30 * M_PI / 180 ||
        i == nodes.size() - judge_dis - 1) {
      if (last_joint > 0) {
        nodes[last_joint].weight *= std::max(nums, last_nums) + 1;
      }
      last_joint = i;
      last_nums = nums;
      nums = 0;
      // std::cout << last_joint << " " << nodes[last_joint].weight <<
      // std::endl;
    } else {
      nums++;
    }
    last_angle = angle;
  }
  nodes[0].weight *= nodes.size() * 2;
  nodes[nodes.size() - 1].weight *= nodes.size() * 2;
}

void PolyFit::initMap(int width, int height, uchar *map) {
  map_width = width;
  map_height = height;
  this->map = map;
}

void PolyFit::calcPolyfitIndepentVariable(double indepent_variable, int n,
                                          Eigen::MatrixXd &A, int index,
                                          float weight, int derivative) {
  for (int i = 0; i < n; i++) {
    int factorial = 1;
    for (int j = 0; j < derivative; j++) {
      for (int k = 1; k <= i - j; k++) {
        factorial *= k;
      }
    }
    if (i < derivative) {
      A(index, i) = 0;
    } else {
      A(index, i) = pow(indepent_variable, i - derivative) * factorial * weight;
    }
  }
}
}  // namespace navigation