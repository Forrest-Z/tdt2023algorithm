/*
 * @Description: 不滤波
 */
#ifndef LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#include "cloud_filter_interface.hpp"

namespace robot_localization {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud_ptr) override;
    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override
    {
      filtered_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(*input_cloud_ptr));
      return true;
    }
    bool Filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& filtered_cloud_ptr) override
    {
      filtered_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZINormal>(*input_cloud_ptr));
      return true;
    }

};
}
#endif