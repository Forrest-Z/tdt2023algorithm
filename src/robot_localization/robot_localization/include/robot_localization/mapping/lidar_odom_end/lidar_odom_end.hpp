/*
 * @Description: 里程计端算法
 */
#ifndef LIDAR_ODOM_END_HPP
#define LIDAR_ODOM_END_HPP

// c++
#include <unordered_map>
#include <string>
// eigen

#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// 点云滤波虚继承接口
#include "localization_models/cloud_filter/cloud_filter_interface.hpp"
// 点云匹配虚继承接口
// 匹配
#include "localization_models/registration/registration_interface.hpp"
#include "localization_models/registration/icp_registration.hpp"
#include "localization_models/registration/icp_svd_registration.hpp"
#include "localization_models/registration/ndt_cpu/ndt_cpu_registration.hpp"
#include "localization_models/registration/ndt_registration.hpp"
#include "localization_models/registration/sicp/scip_registration.hpp"
// 滤波
#include "localization_models/cloud_filter/cloud_filter_interface.hpp"
#include "localization_models/cloud_filter/box_filter.hpp"
#include "localization_models/cloud_filter/voxel_filter.hpp"
#include "localization_models/cloud_filter/no_filter.hpp"
// 融合
#include "../../models/kalman_filter/eskf.hpp"

#include "../../global_path_defination/global_path.h"
#include "localization_tools/tic_toc.hpp"
#include "localization_tools/color_terminal.hpp"
#include "sensor_data/cloud_data.hpp"
namespace robot_localization
{
    
    class LidarOdomEnd
    {
    public:
        struct Frame
        {
            Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
            CloudData cloud_data_;
        };

    public:
        LidarOdomEnd();

        bool Init(const Eigen::Matrix4d &init_pose,
                  const Eigen::Vector3d &init_vel,
                  const ImuData &init_imu_data);

        bool Predict(const ImuData &imu_data);

        bool Correct(const ImuData &imu_data,
                     const CloudData &cloud_data,
                     Eigen::Matrix4d &cloud_pose);

        bool HasInited() const { return has_inited_; }

        double GetTime(void) { return kalman_filter_ptr_->GetTime(); }
        CloudData::CLOUD_PTR &GetCurrentScan() { return current_scan_ptr_; }
        void GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &vel);

        void coordinate_transformation(Eigen::Matrix4d &original_pose,Eigen::Vector3d &change);


    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool ConfigKalmanFilterMethod(std::shared_ptr<KalmanFilterInterface> &kalman_filter_ptr_, const YAML::Node &config_node);

        bool AddNewFrame(const Frame &new_key_frame);

    private:
        std::string data_path_ = "";

        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;
        std::shared_ptr<KalmanFilterInterface> kalman_filter_ptr_;

        std::deque<Frame> local_map_frames_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;

        Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Vector3d current_vel_ = Eigen::Vector3d::Zero();
        Frame current_frame_;

        // 当前滤波器的观测数据
        KalmanFilterInterface::Measurement current_measurement_;

        bool has_inited_ = false;
        float key_frame_distance_ = 2.0;
        int local_frame_num_ = 20;
    };

} // namespace robot_localization

#endif