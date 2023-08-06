/*
 * @Description: 从点云中截取一个立方体部分
 */

#ifndef LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_
#define LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "cloud_filter_interface.hpp"

namespace  robot_localization {
class BoxFilter: public CloudFilterInterface 
{
  public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud_ptr) override;
    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override
    {
      filtered_cloud_ptr->clear();
      pcl_box_filter_2.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
      pcl_box_filter_2.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
      pcl_box_filter_2.setInputCloud(input_cloud_ptr);
      pcl_box_filter_2.filter(*filtered_cloud_ptr);

    return true;
    }
    bool Filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& filtered_cloud_ptr) override
    {
      filtered_cloud_ptr->clear();
      pcl_box_filter_3.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
      pcl_box_filter_3.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
      pcl_box_filter_3.setInputCloud(input_cloud_ptr);
      pcl_box_filter_3.filter(*filtered_cloud_ptr);

    return true;
    }


    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<pcl::PointXYZ> pcl_box_filter_;
    pcl::CropBox<pcl::PointXYZI> pcl_box_filter_2;
    pcl::CropBox<pcl::PointXYZINormal> pcl_box_filter_3;


    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}

#endif 