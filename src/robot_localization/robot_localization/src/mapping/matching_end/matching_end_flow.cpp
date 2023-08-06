/*
 * @Description: 地图匹配任务管理
 */
// #include "../../../include/mapping/matching_end/matching_end_flow.hpp"
#include "robot_localization/mapping/matching_end/matching_end_flow.hpp"
#include "glog/logging.h"

namespace  robot_localization 
{
MatchingFlow::MatchingFlow(std::shared_ptr<rclcpp::Node>& node_) 
{   
    std::string yaml_config_path = node_->declare_parameter<std::string>("yaml_config_path", WORK_PACKAGE_PATH + "/config/livox_lidar.yaml");
    
    YAML::Node user_config = YAML::Node();
    try {
        user_config = YAML::LoadFile(yaml_config_path);
        std::cout<<"yaml载入配置文件："<<yaml_config_path<<"\n";
    } catch (YAML::Exception& e) 
    {
        RCLCPP_ERROR(node_->get_logger(), "无法加载 YAML config: %s，寻找不到文件", e.what());
        return;
    }

    // 配置用户设置消息话题
    std::string global_frame_id;
    std::string undistorted_pointcloud_topic;
    std::string lidar_link;
    std::string car_base_link;

    try {
    global_frame_id = user_config["global_frame_id"].as<std::string>();
    lidar_link = user_config["lidar_link"].as<std::string>();
    car_base_link = user_config["car_base_link"].as<std::string>();
    undistorted_pointcloud_topic = user_config["undistorted_pointcloud"].as<std::string>();
    if_matching_end_tf_broadcast=user_config["matching_end_send_tf"].as<bool>();

    coordinate_transformation = Eigen::Vector3d(
                                                    user_config["init_position"][0].as<double>(),
                                                    user_config["init_position"][1].as<double>(),
                                                    user_config["init_position"][2].as<double>()
                                                   );
    } catch (YAML::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "获取参数时出现错误，可能参数类型或者不存在该参数， config: %s", e.what());
        RCLCPP_WARN(node_->get_logger(),"将赋值设定参数,这是非正常的操作，即使正常运行，请检查修复");
        global_frame_id = "map";
        lidar_link = "laser_link";
        car_base_link = "laser_link";
        undistorted_pointcloud_topic = "/livox/lidar_pro";
        if_matching_end_tf_broadcast = false;
        coordinate_transformation = Eigen::Vector3d(0.00,0.00,0.00);
    }


    // 订阅:
    // 已去畸变的点云: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(node_, undistorted_pointcloud_topic, 100000);

    // 发布:
    // 1. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "global_map", global_frame_id, 100);
    // 2. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "local_map", global_frame_id, 100);
    // 3. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "current_scan", global_frame_id, 100);
    // 4. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(node_, "map_matching_odom", global_frame_id, "lidar", 100);
    // 5. tf
    laser_tf_pub_ptr_ = std::make_shared<TFBroadcaster>(node_,global_frame_id, car_base_link);

    //matching任务管理器，构造函数会为其配置yaml参数内容，以及加载全局点云pcd文件并更新一次局部地图
    matching_ptr_ = std::make_shared<Matching>(user_config);
}

bool MatchingFlow::Run() 
{
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) 
    {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData();
    // green_info("Match ReadDate Done");
    while(HasData()) 
    {
        if (!ValidData())
        {
            LOG(INFO) << "Invalid data 跳过匹配" << std::endl;
            continue;
        }

        if (UpdateMatching()) 
        {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() 
{
    // 将激光雷达测量结果送入缓冲区
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
        
    return true;
}

bool MatchingFlow::ValidData() 
{
    current_cloud_data_ = cloud_data_buff_.front();

    if (matching_ptr_->HasInited()) 
    {
        cloud_data_buff_.pop_front();

        return true;
    }


    cloud_data_buff_.pop_front();

    return true;
}

/**
 * @brief 更新匹配结果
 * @param 
 **/
bool MatchingFlow::UpdateMatching() 
{
    if (!matching_ptr_->HasInited())    // 第一帧点云数据，在此处实现位姿全局初始化
    {                
        // 首次进行图匹配时，希望获取较为准确的起始位姿，若首次投入位姿不能超过2m，角度必须在+-40否则图匹配难以处理投入点云

        /*地图原点初始化，置 init_pose  为单位阵*/// 注释此为天真（naive）的方法
        Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();     

        // 手动设置，都是设置为 0 0 0，需要物理世界中与建图时候的起点一致
        matching_ptr_->SetInitPose(init_pose);
        
        /*或者，利用ScanContext 进行位姿初始化*/
        // matching_ptr_->SetScanContextPose(current_cloud_data_);

        // matching_ptr_->SetInited();
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::PublishData() 
{   
    if(if_matching_end_tf_broadcast)    
        laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.ros2_time);
    matching_ptr_->coordinate_transformation(laser_odometry_ ,coordinate_transformation);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.ros2_time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}