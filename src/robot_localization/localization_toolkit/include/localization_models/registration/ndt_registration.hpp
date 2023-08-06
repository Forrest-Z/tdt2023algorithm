/*
 * @Description: NDT匹配算法
 */

#ifndef NDT_REGISTRATION_HPP
#define NDT_REGISTRATION_HPP

#include "registration_interface.hpp"
// pcl
#include <pcl/registration/ndt.h>
// yaml
#include <yaml-cpp/yaml.h>
// glog
#include <glog/logging.h>

namespace robot_localization
{
    class NdtRegistration : public RegistrationInterface
    {
    public:
        NdtRegistration(const YAML::Node &node);
        NdtRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override;
        bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr,
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
      ;
    } 
    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_ptr_;
    };

} // namespace robot_localization

#endif