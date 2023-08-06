/*
 * @Description: ICP 匹配模块
 */
#ifndef ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include "registration_interface.hpp"
// pcl
#include <pcl/registration/icp.h>
#include "glog/logging.h"

namespace robot_localization
{
  class ICPRegistration : public RegistrationInterface
  {
  public:
    ICPRegistration(const YAML::Node &node): icp_ptr_(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
    {
        float max_corr_dist = node["max_corr_dist"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    ICPRegistration(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter): icp_ptr_(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
    {
        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override
    {
        icp_ptr_->setInputTarget(input_target);

        return true;
    }
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override
    {
        icp_ptr_->setInputSource(input_source);           //  输入待配准点云
        icp_ptr_->align(*result_cloud_ptr, predict_pose.cast<float>()); // 配准
        result_pose = icp_ptr_->getFinalTransformation().cast<double>(); // 获取变换矩阵

        return true;
    }
    bool ScanMatch(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                const Eigen::Matrix4f &guess_pose,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose,
                                float &fitness_score
                              ) override
    {
      icp_ptr_->setInputSource(input_source);           //  输入待配准点云
      icp_ptr_->align(*result_cloud_ptr, guess_pose); // 配准
      result_pose = icp_ptr_->getFinalTransformation(); // 获取变换矩阵
      fitness_score = (float)icp_ptr_->getFitnessScore();
      
      return true;
    } 

  private:
    bool SetRegistrationParam(
                              float max_corr_dist,float trans_eps,
                              float euc_fitness_eps,int max_iter
                             )
    {
        icp_ptr_->setMaxCorrespondenceDistance(max_corr_dist);
        icp_ptr_->setTransformationEpsilon(trans_eps);
        icp_ptr_->setEuclideanFitnessEpsilon(euc_fitness_eps);
        icp_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << "ICP params:" << std::endl
                  << "max_corr_dist: " << max_corr_dist << ", "
                  << "trans_eps: " << trans_eps << ", "
                  << "euc_fitness_eps: " << euc_fitness_eps << ", "
                  << "max_iter: " << max_iter
                  << std::endl
                  << std::endl;

        return true;
    }

  private:
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_ptr_;
  };
}

#endif