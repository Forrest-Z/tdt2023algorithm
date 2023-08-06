/*
 * @Description: 不滤波
 */
// #include "../../../include/models/cloud_filter/no_filter.hpp"
#include "localization_models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace robot_localization {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud_ptr) 
{
    filtered_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud_ptr));
    return true;
}
} 