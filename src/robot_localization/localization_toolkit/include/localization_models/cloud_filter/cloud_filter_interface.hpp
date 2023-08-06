/*
 * @Description: 点云滤波接口
 */
#ifndef CLOUD_FILTER_INTERFACE_HPP
#define CLOUD_FILTER_INTERFACE_HPP
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// yaml
#include <yaml-cpp/yaml.h>

namespace robot_localization
{
    class CloudFilterInterface
    {
    public:
        virtual ~CloudFilterInterface() = default;
        virtual bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) = 0;
        virtual bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud_ptr) = 0;
        virtual bool Filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &filtered_cloud_ptr) = 0;
    };

} // namespace robot_localization

#endif