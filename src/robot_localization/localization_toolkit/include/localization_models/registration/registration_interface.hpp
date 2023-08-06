/*
 * @Description: 点云匹配接口
 */
#ifndef REGISTRATION_INTERFACE_HPP
#define REGISTRATION_INTERFACE_HPP

// 自定义点云消息类型
// yaml
#include <yaml-cpp/yaml.h>
// eigen
#include <Eigen/Dense>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace robot_localization
{
    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;
        virtual bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) = 0;
        virtual bool ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr,
                               const Eigen::Matrix4d &predict_pose,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                               Eigen::Matrix4d &result_pose) = 0;
        virtual bool ScanMatch(
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source,
                                const Eigen::Matrix4f &guess_pose,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose,
                                float &fitness_score
                              ) = 0;
        
        virtual float GetFitnessScore()
        {
            std::cout<<"!!!该匹配方式无GetFitnessScore function!!!\n";
            return 0;
        }
    };

} // namespace robot_localization

#endif