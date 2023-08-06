/*
 * @Description: SICP registration
 */
#ifndef MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#define MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#include <pcl/common/transforms.h>
#include "../registration_interface.hpp"
#include "../sicp/ICP.h"
namespace robot_localization
{

  class SICPRegistration : public RegistrationInterface
  {
  public:
    SICPRegistration(const YAML::Node &node);

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override;
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_source_;

    Eigen::Matrix4d transformation_;
    SICP::Parameters params_;
  };

} // namespace robot_localization

#endif