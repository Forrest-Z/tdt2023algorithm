/*
 * @Description: 点云配准打分机制，评估该帧定位可信度
 */

#ifndef LOCALIZATION_MODELS_REGISTRATION_SCORE_HPP_
#define LOCALIZATION_MODELS_REGISTRATION_SCORE_HPP_

#include <pcl/filters/crop_box.h>
#include <sensor_msgs/msg/imu.hpp>
#include "localization_models/ikd-Tree/ikd_Tree.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
/**
 * @brief 建图加入点云信息
 * @param 输入ikdtree类对象的（智能）指针，已经转换到了世界坐标系（起点坐标系）下的点云
 * @param 输出的参数以引用形式返回：
 **/
inline void ComputeScore (
    //   KD_TREE<pcl::PointXYZINormal>::Ptr ikdtree,
      KD_TREE<pcl::PointXYZINormal>& ikdtree,   // 引用要比智能指针还快
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr measure_world_cloud,
      double &average_distance_score,
      double &overlap_score
    //   Eigen::Matrix4f &transform
                           )
{
    double total_distance = 0.0;
    int overlap_count = 0;

    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // // pcl::transformPointCloud(*measure_cloud, *registered_cloud, transform);

    // Compute the distance measure
    for (const auto& point : measure_world_cloud->points) {
        // Find the nearest point in the IKD-Tree
        std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> nearest_points;
        vector<float> nearest_points_distances;
        ikdtree.Nearest_Search(point, 1, nearest_points, nearest_points_distances, 1.26);

        // If no nearest point was found, continue to the next point
        if (nearest_points.empty()) {
            continue;
        }

        // Compute the distance
        double distance = nearest_points_distances.front();
        total_distance += distance;

        // Count the number of points that are closer than the threshold
        float threshold = 0.005;
        if (distance < threshold) {
            overlap_count++;
        }
    }

    double average_distance = total_distance / measure_world_cloud->points.size();
    double overlap_ratio = static_cast<double>(overlap_count) / measure_world_cloud->points.size();

    // Compute the final score based on your weighting scheme
    float weight_distance = 1.0;
    float weight_overlap = 1.0;
    // double score = weight_distance * average_distance + weight_overlap * overlap_ratio;
    average_distance_score = weight_distance * average_distance;
    overlap_score = weight_overlap * overlap_ratio;

}


#endif 