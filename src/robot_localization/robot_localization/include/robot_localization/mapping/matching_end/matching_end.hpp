/*
 * @Description: 地图匹配定位算法
 */
#ifndef LOCALIZATION_MATCHING_MATCHING_END_HPP_
#define LOCALIZATION_MATCHING_MATCHING_END_HPP_

#include <deque>
#include "localization_models/registration/registration_interface.hpp" //包含CloudDate
#include "localization_models/cloud_filter/cloud_filter_interface.hpp"
#include "localization_models/cloud_filter/no_filter.hpp"
#include "localization_models/cloud_filter/box_filter.hpp"
#include "localization_models/cloud_filter/voxel_filter.hpp"

#include "localization_models/registration/icp_registration.hpp"
#include "localization_models/registration/icp_svd_registration.hpp"
#include "localization_models/registration/ndt_cpu/ndt_cpu_registration.hpp"
#include "localization_models/registration/ndt_registration.hpp"
#include "localization_models/registration/sicp/scip_registration.hpp"

#ifdef BUILDING_WITH_CUDA  // 来自CmakeList的指定
#include "localization_models/registration/ndt_gpu/ndt_gpu_registration.hpp"
#include "localization_models/registration/ndt_cuda/ndt_cuda_registration.hpp"
#include "localization_models/registration/icp_cuda/icp_cuda_registration.hpp"
#endif
#include "localization_tools/tic_toc.hpp"
#include "../../global_path_defination/global_path.h"
#include "sensor_data/cloud_data.hpp"
// #include "../../models/scan_context_manager/scan_context_manager.hpp"//处理优化端，暂时注释。别忘了放开
#include "rclcpp/rclcpp.hpp"
namespace  robot_localization 
{
class Matching {
  public:
    Matching(const YAML::Node& user_config);

    bool Update(const CloudData& cloud_data, Eigen::Matrix4d& cloud_pose);

    bool SetScanContextPose(const CloudData& init_scan); //

    bool SetInitPose(const Eigen::Matrix4d& init_pose);
    bool SetInited(void);

    Eigen::Matrix4d GetInitPose(void);
    void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
    CloudData::CLOUD_PTR& GetLocalMap();
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();

  private:
    bool InitWithConfig(const YAML::Node& user_config);

    bool InitScanContextManager(const YAML::Node& config_node);  
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitBoxFilter(const YAML::Node& config_node);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string map_path_ = "";
    std::string registration_method="";

    // std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;//处理优化端，暂时注释。别忘了放开
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;
    
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
    public:
    /**
     * @brief  坐标变换
     * @note   输入original_pose，用change给出的三个坐标变换相加
     * @todo
     **/
    void coordinate_transformation(Eigen::Matrix4d &original_pose, Eigen::Vector3d &change)
    {   
        original_pose(0, 3) += change[0];
        original_pose(1, 3) += change[1];
        original_pose(2, 3) += change[2];

    }
};
}

#endif