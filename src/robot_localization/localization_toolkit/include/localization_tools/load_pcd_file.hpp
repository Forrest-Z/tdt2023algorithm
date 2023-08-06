/*
 * @Description: 
 */
#ifndef FAST_LOAD_PCD_FILE_HPP_
#define FAST_LOAD_PCD_FILE_HPP_
#pragma once
#include <iostream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <pcl/types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <omp.h>

namespace robot_localization
{

    inline void one_load_pointcloud1(const std::string& filepath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
    {
        if (pcl::io::loadPCDFile(filepath, *cloud) == -1) 
        {
            std::cout << "Couldn't read file " << filepath << std::endl;
        }
        
        // pcl::PCDReader reader;// 创建 PCDReader 对象
        // // 从文件中读取点云数据
        // if (reader.read<pcl::PointXYZI>(filepath, *cloud) == -1) 
        // {
        //     PCL_ERROR("Couldn't read file input_cloud.pcd\n");
        // }

    }
    inline void one_load_pointcloud2(const std::string& filepath, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud) 
    {
        if (pcl::io::loadPCDFile(filepath, *cloud) == -1) 
        {
            std::cout << "Couldn't read file " << filepath << std::endl;
        }
        // pcl::PCDReader reader;// 创建 PCDReader 对象
        // // 从文件中读取点云数据
        // if (reader.read<pcl::PointXYZINormal>(filepath, *cloud) == -1) 
        // {
        //     PCL_ERROR("Couldn't read file input_cloud.pcd\n");
        // }
    }
    inline void LoadPointCloudsFiles(const std::vector<std::string>& filepaths,pcl::PointCloud<pcl::PointXYZI>::Ptr input_also_output) 
    {
        size_t num_files = filepaths.size();
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds(num_files);

#pragma omp parallel for
        for (size_t i = 0; i < num_files; ++i) 
        {
            clouds[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            one_load_pointcloud1(filepaths[i], clouds[i]);
        }

        input_also_output->clear();
        size_t total_points = 0;
        for (const auto& one_cloud : clouds) 
        {
            total_points += one_cloud->size();
        }
        input_also_output->reserve(total_points);

        for (const auto& one_cloud : clouds) 
        {
            *input_also_output += *one_cloud; 
        }

    }

    inline void LoadPointCloudsFiles(const std::vector<std::string>& filepaths,pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_also_output) 
    {
        size_t num_files = filepaths.size();
        std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds(num_files);
#pragma omp parallel for
        for (size_t i = 0; i < num_files; ++i) 
        {
            clouds[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
            one_load_pointcloud2(filepaths[i], clouds[i]);
        }
        input_also_output->clear();
        size_t total_points = 0;
        for (const auto& one_cloud : clouds) 
        {
            total_points += one_cloud->size();
        }
        input_also_output->reserve(total_points);

        for (const auto& one_cloud : clouds) 
        {
            *input_also_output += *one_cloud; 
        }

    }
  

} // namespace robot_localization
#endif
