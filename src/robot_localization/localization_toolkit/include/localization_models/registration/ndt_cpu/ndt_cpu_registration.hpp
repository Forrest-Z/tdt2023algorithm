#ifndef REGISTRATION_NDT_CPU_REGISTRATION_HPP_
#define REGISTRATION_NDT_CPU_REGISTRATION_HPP_

#include "../registration_interface.hpp"
#include "NormalDistributionsTransform.h"

namespace robot_localization
{

  class NDTCPURegistration : public RegistrationInterface// 继承点云配准的基类
  { 
  public:
    NDTCPURegistration(const YAML::Node &node);

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override;
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override;
    float GetFitnessScore() override;
    bool ScanMatch(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                const Eigen::Matrix4f &guess_pose,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose,
                                float &fitness_score
                              ) override
    {
        // TicToc example1;
        ndt_cpu_.setInputSource(input_source);
        // printf("TicToc setInputSource耗时 %.6lf ms\n",example1.toc());
        // TicToc example2;
        ndt_cpu_.align(*result_cloud_ptr, guess_pose);  // 配准用的float，阿这
        // printf("TicToc align耗时 %.6lf ms\n",example2.toc());
        // TicToc example3;
        result_pose = ndt_cpu_.getFinalTransformation(); // 匹配后的点云
        // printf("TicToc getFinalTransformation耗时 %.6lf ms\n",example3.toc());
        fitness_score = ndt_cpu_.getFitnessScore();


        return true;
    } 
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_cpu_; // 实例化cpu_ndt 对象
  };

} // namespace  robot_localization

#endif