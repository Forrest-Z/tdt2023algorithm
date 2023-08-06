// // 增采样是一种表面重建方法，当有比你想象的要少的点云数据时
// //   增采样可以帮助恢复原有的表面（S），通过内插你目前拥有的点云数据，
// //   这是一个复杂的猜想假设的过程。所以构建的结果不会百分之一百准确，
// //   但有时它是一种可选择的方案。
// #ifndef VOXEL_FILTER_HPP
// #define VOXEL_FILTER_HPP

// #include "cloud_filter_interface.hpp"
// #include <pcl/surface/mls.h>

// namespace robot_localization
// {
//     class Upsampling : public CloudFilterInterface
//     {
//     public:
//         Upsampling(const YAML::Node &node)
//         {
//             float leaf_size_x = node["leaf_size"][0].as<float>();
//             float leaf_size_y = node["leaf_size"][1].as<float>();
//             float leaf_size_z = node["leaf_size"][2].as<float>();

//             SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
//         }

//     private:
//         bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
//         {
//             mls_1
//             LOG(INFO) << "[VoxelFilter_param]" << std::endl
//                     << leaf_size_x << "," << leaf_size_y << "," << leaf_size_z << std::endl;
//             return true;
//         }

//     private:
//         pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_1;
//         pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls_2;
//         pcl::MovingLeastSquares<pcl::PointXYZINormal, pcl::PointXYZINormal> mls_3; 
//     };
    
// }

// #endif
