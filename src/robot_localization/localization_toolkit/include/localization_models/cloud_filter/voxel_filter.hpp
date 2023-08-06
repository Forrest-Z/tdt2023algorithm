/*
 * @Description: 体素滤波器
 */

#ifndef VOXEL_FILTER_HPP
#define VOXEL_FILTER_HPP

#include "cloud_filter_interface.hpp"
// glog
#include <glog/logging.h>
// pcl
#include <pcl/filters/voxel_grid.h>

namespace robot_localization
{
    class VoxelFilter : public CloudFilterInterface
    {
    public:

        /**
        * @brief 体素滤波器初始化
        * @note YAML node设定体素滤波格子大小参数，单位：米
        **/
        VoxelFilter(const YAML::Node &node)
        {
            float leaf_size_x = node["leaf_size"][0].as<float>();
            float leaf_size_y = node["leaf_size"][1].as<float>();
            float leaf_size_z = node["leaf_size"][2].as<float>();

            SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
        }

        /**
        * @brief 体素滤波器初始化
        * @note 形参传入方式设定参数
        **/
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
        {
            SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
        }

        bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) override
        {
            voxel_filter_1.setInputCloud(input_cloud_ptr);
            voxel_filter_1.filter(*filtered_cloud_ptr);

            return true;
        }
        bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr &filter_cloud_ptr) override
        {
            voxel_filter_2.setInputCloud(input_cloud_ptr);
            voxel_filter_2.filter(*filter_cloud_ptr);

            return true;
        }
        bool Filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &filter_cloud_ptr) override
        {
            voxel_filter_3.setInputCloud(input_cloud_ptr);
            voxel_filter_3.filter(*filter_cloud_ptr);

            return true;
        }

    private:
        /**
         * @brief 设置滤波参数
         * @note
         **/
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
        {
            voxel_filter_1.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
            voxel_filter_2.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
            voxel_filter_3.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
            LOG(INFO) << "[VoxelFilter_param]" << std::endl
                    << leaf_size_x << "," << leaf_size_y << "," << leaf_size_z << std::endl;
            return true;
        }

    private:
        pcl::VoxelGrid<pcl::PointXYZ>        voxel_filter_1;
        pcl::VoxelGrid<pcl::PointXYZI>       voxel_filter_2;
        pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter_3;

    };

} // namespace robot_localization

#endif
