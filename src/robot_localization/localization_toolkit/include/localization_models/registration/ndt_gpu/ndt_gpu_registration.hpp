#ifndef REGISTRATION_NDT_GPU_REGISTRATION_HPP_
#define REGISTRATION_NDT_GPU_REGISTRATION_HPP_

#include "../registration_interface.hpp"
#include "NormalDistributionsTransform.h"
#include <pcl/io/pcd_io.h>

namespace robot_localization
{

  class NDTGPURegistration : public RegistrationInterface// 继承点云配准的基类
  { 
  public:
    NDTGPURegistration(const YAML::Node &node)
    {   
        input_target_pc.reset( new pcl::PointCloud<pcl::PointXYZ>());
        float res = node["res"].as<float>();
        double step_size = node["step_size"].as<double>();
        double trans_eps = node["trans_eps"].as<double>();
        int max_iter = node["max_iter"].as<int>();

        ndt_gpu_.setMaximumIterations(max_iter);      //迭代最大次数
        ndt_gpu_.setResolution(res);                  //网格大小设置
        ndt_gpu_.setTransformationEpsilon(trans_eps);  //连续变换之间允许的最大差值
        ndt_gpu_.setStepSize(step_size);             //牛顿法优化的最大步长
    }

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override
    {
        // ndt_gpu_.setInputTarget(input_target);
        pcl::copyPointCloud(*input_target, *input_target_pc);

        return true;
    }
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override
    {
        // TicToc example1;
        ndt_gpu_.setInputSource(input_source);
        ndt_gpu_.setInputTarget(input_target_pc);
        // printf("TicToc setInputSource耗时 %.6lf ms\n",example1.toc());
        // TicToc example2;
        ndt_gpu_.align(predict_pose.cast<float>());  // 配准用的float，阿这
        // printf("TicToc align耗时 %.6lf ms\n",example2.toc());
        // TicToc example3;
        // std::cout<<"ssdhf"<<"\n";
        result_pose = ndt_gpu_.getFinalTransformation().cast<double>(); // 匹配后的点云
        // printf("TicToc getFinalTransformation耗时 %.6lf ms\n",example3.toc());

        return true;

    }
    float GetFitnessScore() override
    {
        return ndt_gpu_.getFitnessScore();
    }
    bool ScanMatch(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                const Eigen::Matrix4f &guess_pose,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose,
                                float &fitness_score
                              ) override
    {
      ;
    }

  private:
    gpu::GNormalDistributionsTransform ndt_gpu_; // 实例化cpu_ndt 对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_pc;
  };

} // namespace  robot_localization

#endif