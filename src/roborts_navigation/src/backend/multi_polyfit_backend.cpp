#include "multi_polyfit_backend.h"
namespace navigation {
MultiPolyfitBackend::Result MultiPolyfitBackend::solve(
    std::vector<Node> &nodes,  // 前端路径点
    std::pair<tdttoolkit::Vec2d, tdttoolkit::Vec2d>
        velocity,      // 起始点和终点速度约束
    int order,         // 多项式阶数
    int resolve_rate,  // 分段解析度(每段包含多少点)
    double acceleration,  // 预估分段时间时所用加速度(应小于实际最大加速度)
    double max_velocity  // 全局最大速度(无法约束到具体点)
) {
  int segments_nums = (nodes.size() - 1) / resolve_rate + 1;  // 分段段数
  Result result;                                              // 储存结果

  double cnt_forward_speed =  // 当前向前规划的速度
      std::sqrt(velocity.first.x * velocity.first.x +
                velocity.first.y * velocity.first.y);
  double cnt_backward_speed =  // 当前向后规划的速度
      std::sqrt(velocity.second.x * velocity.second.x +
                velocity.second.y * velocity.second.y);
  double cnt_forward_time = 0,  // 当前向前规划的时间
      cnt_backward_time = 0;    //  当前向后规划的时间

  // 当前向前规划的位置
  int cnt_forward_position = 0;
  // 当前向后规划的位置
  int cnt_backward_position = nodes.size() - 1;

  while (cnt_forward_position < cnt_backward_position) {
    // 判断当前向前规划还是向后规划
    bool forward = cnt_forward_speed <= cnt_backward_speed;

    int start_point =  // 当前段起点(闭区间)
        forward ? cnt_forward_position : cnt_backward_position - resolve_rate;
    int end_point =  // 当前段终点(闭区间)
        forward ? cnt_forward_position + resolve_rate : cnt_backward_position;

    bool meet = false;  // 是否相遇
    if (forward) {
      if (cnt_forward_position + resolve_rate >= cnt_backward_position) {
        end_point = cnt_backward_position;
        meet = true;
      }
      cnt_forward_position = end_point;
    } else {
      if (cnt_backward_position - resolve_rate <= cnt_forward_position) {
        start_point = cnt_forward_position;
        meet = true;
      }
      cnt_backward_position = start_point;
    }

    double cnt_start_angle = 0;
    if (start_point == 0) {
      cnt_start_angle = atan2(velocity.first.y, velocity.first.x);

    } else {
      cnt_start_angle = calcSpeedDirection(
          nodes[start_point - 1], nodes[start_point], nodes[start_point + 1]);
    }
    double cnt_end_angle = 0;
    if (end_point == nodes.size() - 1) {
      cnt_end_angle = atan2(velocity.second.y, velocity.second.x);
    } else {
      cnt_end_angle = calcSpeedDirection(nodes[end_point - 1], nodes[end_point],
                                         nodes[end_point + 1]);
    }

    double cnt_segment_time = 0;         // 当前分段的时间
    int weight_start_end_position = 10;  // 起点终点位置的权重
    int weight_current_speed = 0.1;      // 当前速度的权重
    int weight_target_speed = 0.1;       // 目标速度的权重
    int weight_midpoint_position = 1;    // 中间点位置的权重

    if (forward) {
      cnt_segment_time = calcSegmentTime(nodes[start_point], nodes[end_point],
                                         acceleration, cnt_start_angle,
                                         cnt_forward_speed);  // 计算当前段时间
    } else {
      cnt_segment_time =
          calcSegmentTime(nodes[end_point], nodes[start_point], acceleration,
                          cnt_end_angle + CV_PI, cnt_backward_speed);
    }
    int size = end_point - start_point + 1;  // 当前段的点数

    // 用于存储x方向的约束矩阵自变量
    Eigen::MatrixXd constraint_xtorword_x(size + 2, order + 1);
    // 用于存储x方向的约束矩阵因变量
    Eigen::MatrixXd constraint_xtorword_y(size + 2, 1);
    // 用于存储y方向的约束矩阵自变量
    Eigen::MatrixXd constraint_ytorword_x(size + 2, order + 1);
    // 用于存储y方向的约束矩阵因变量
    Eigen::MatrixXd constraint_ytorword_y(size + 2, 1);

    // 添加路径点的位置约束
    for (int i = start_point; i <= end_point; i++) {
      int weight = weight_midpoint_position;  // 当前点的权重
      if (i == start_point || i == end_point)
        weight = weight_start_end_position;
      double t = (i - start_point) / (double)size * cnt_segment_time;
      for (int j = 0; j <= order; j++) {
        constraint_xtorword_x(i - start_point, j) = pow(t, j) * weight;
        constraint_ytorword_x(i - start_point, j) = pow(t, j) * weight;
      }
      constraint_xtorword_y(i - start_point, 0) = nodes[i].x * weight;
      constraint_ytorword_y(i - start_point, 0) = nodes[i].y * weight;
    };

    // 添加速度约束
    if (forward) {
      for (int i = 1; i <= order; i++) {
        constraint_xtorword_x(size, i) =
            i * pow(0, i - 1) * weight_current_speed;
        constraint_ytorword_x(size, i) =
            i * pow(0, i - 1) * weight_current_speed;
        if (meet) {
          constraint_xtorword_x(size + 1, i) =
              i * pow(cnt_segment_time, i - 1) * weight_current_speed;
          constraint_ytorword_x(size + 1, i) =
              i * pow(cnt_segment_time, i - 1) * weight_current_speed;
        } else {
          constraint_xtorword_x(size + 1, i) =
              i * pow(cnt_segment_time, i - 1) * weight_target_speed;
          constraint_ytorword_x(size + 1, i) =
              i * pow(cnt_segment_time, i - 1) * weight_target_speed;
        }
      }
      constraint_xtorword_y(size, 0) =
          cnt_forward_speed * cos(cnt_start_angle) * weight_current_speed;
      constraint_ytorword_y(size, 0) =
          cnt_forward_speed * sin(cnt_start_angle) * weight_current_speed;
      if (meet) {
        constraint_xtorword_y(size + 1, 0) =
            cnt_backward_speed * cos(cnt_end_angle) * weight_current_speed;
        constraint_ytorword_y(size + 1, 0) =
            cnt_backward_speed * sin(cnt_end_angle) * weight_current_speed;
      } else {
        constraint_xtorword_y(size + 1, 0) =
            max_velocity * cos(cnt_end_angle) * weight_target_speed;
        constraint_ytorword_y(size + 1, 0) =
            max_velocity * sin(cnt_end_angle) * weight_target_speed;
      }
    } else {
      for (int i = 1; i <= order; i++) {
        if (meet) {
          constraint_xtorword_x(size, i) =
              i * pow(0, i - 1) * weight_current_speed;
          constraint_ytorword_x(size, i) =
              i * pow(0, i - 1) * weight_current_speed;
        } else {
          constraint_xtorword_x(size, i) =
              i * pow(0, i - 1) * weight_target_speed;
          constraint_ytorword_x(size, i) =
              i * pow(0, i - 1) * weight_target_speed;
        }
        constraint_xtorword_x(size + 1, i) =
            i * pow(cnt_segment_time, i - 1) * weight_current_speed;
        constraint_ytorword_x(size + 1, i) =
            i * pow(cnt_segment_time, i - 1) * weight_current_speed;
      }
      if (meet) {
        constraint_xtorword_y(size, 0) =
            cnt_forward_speed * cos(cnt_start_angle) * weight_current_speed;
        constraint_ytorword_y(size, 0) =
            cnt_forward_speed * sin(cnt_start_angle) * weight_current_speed;
      } else {
        constraint_xtorword_y(size, 0) =
            max_velocity * cos(cnt_start_angle) * weight_target_speed;
        constraint_ytorword_y(size, 0) =
            max_velocity * sin(cnt_start_angle) * weight_target_speed;
      }
      constraint_xtorword_y(size + 1, 0) =
          cnt_backward_speed * cos(cnt_end_angle) * weight_current_speed;
      constraint_ytorword_y(size + 1, 0) =
          cnt_backward_speed * sin(cnt_end_angle) * weight_current_speed;
    }
    Eigen::MatrixXd result_x =
        constraint_xtorword_x.colPivHouseholderQr().solve(
            constraint_xtorword_y);
    Eigen::MatrixXd result_y =
        constraint_ytorword_x.colPivHouseholderQr().solve(
            constraint_ytorword_y);
    if (forward) {
      result.addForward(result_x, result_y,
                        cnt_forward_time + cnt_segment_time);
      cnt_forward_time += cnt_segment_time;
      double end_speed_x = 0, end_speed_y = 0;  // 当前段优化后的终点速度
      for (int i = 1; i <= order; i++) {
        end_speed_x += i * result_x(i, 0) * pow(cnt_segment_time, i - 1);
        end_speed_y += i * result_y(i, 0) * pow(cnt_segment_time, i - 1);
      }
      cnt_forward_speed =
          sqrt(end_speed_x * end_speed_x + end_speed_y * end_speed_y);

    } else {
      result.addBackward(result_x, result_y,
                         cnt_backward_time + cnt_segment_time);
      cnt_backward_time += cnt_segment_time;
      double start_speed_x = 0, start_speed_y = 0;  // 当前段优化后的起点速度
      for (int i = 1; i <= order; i++) {
        start_speed_x += i * result_x(i, 0) * pow(0, i - 1);
        start_speed_y += i * result_y(i, 0) * pow(0, i - 1);
      }
      cnt_backward_speed =
          sqrt(start_speed_x * start_speed_x + start_speed_y * start_speed_y);
    }
  }
  return result;
}

double MultiPolyfitBackend::calcSpeedDirection(const Node &start,
                                               const Node middle,
                                               const Node &end) {
  double start_angle = atan2(start.y - middle.y, start.x - middle.x);
  double end_angle = atan2(end.y - middle.y, end.x - middle.x);
  double angle = (start_angle + end_angle) / 2;
  return angle;
}

double MultiPolyfitBackend::calcSegmentTime(const Node &start, const Node &end,
                                            const double acceleration,
                                            const double start_angle,
                                            const double start_speed) {
  double x_dis = end.x - start.x;
  double x_speed = cos(start_angle) * start_speed;
  double supposed_t_x = fabs(
      (-x_speed + sqrt(x_speed * x_speed + fabs(2 * acceleration * x_dis))) /
      acceleration);
  double y_dis = end.y - start.y;
  double y_speed = sin(start_angle) * start_speed;
  double supposed_t_y = fabs(
      (-y_speed + sqrt(y_speed * y_speed + fabs(2 * acceleration * y_dis))) /
      acceleration);
  return std::max(supposed_t_x, supposed_t_y);
}
}  // namespace navigation