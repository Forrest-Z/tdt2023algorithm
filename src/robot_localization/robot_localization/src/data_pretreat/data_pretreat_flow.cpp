/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 */
#include "robot_localization/data_pretreat/data_pretreat_end_flow.hpp"
#include "robot_localization/global_path_defination/global_path.h"
#include "localization_tools/color_terminal.hpp"
#include <pcl/filters/passthrough.h>

#include "glog/logging.h"

namespace robot_localization {

DataPretreatFlow::DataPretreatFlow(std::shared_ptr<rclcpp::Node>& node_) :ros2_node(node_)
{       
        std::string yaml_config_path = node_->declare_parameter<std::string>("yaml_config_path", WORK_PACKAGE_PATH + "/config/livox_lidar.yaml");
        // 读取YAML参数
        // YAML::Node user_config = YAML::LoadFile(WORK_PACKAGE_PATH + "/config/user_setting.yaml");
        YAML::Node user_config = YAML::LoadFile(yaml_config_path);

        // 配置消息话题
        std::string imu_raw_data_topic="";
        std::string raw_pointcloud_topic="";
        std::string imu_link="";
        std::string lidar_link="";
        std::string car_base_link="";
        std::string undistorted_pointcloud_topic="";
        std::string global_frame_id;
        std::string imu_synced_data_topic;
        int lidar_type;
        lidar_type = user_config["lidar_type"].as<int>();
        global_frame_id = user_config["global_frame_id"].as<std::string>();
        imu_raw_data_topic = user_config["imu"].as<std::string>();
        raw_pointcloud_topic = user_config["lidar_pointcloud"].as<std::string>();
        imu_link = user_config["imu_link"].as<std::string>();
        lidar_link = user_config["lidar_link"].as<std::string>();
        car_base_link = user_config["car_base_link"].as<std::string>();
        imu_synced_data_topic = user_config["imu_synced"].as<std::string>();
        undistorted_pointcloud_topic = user_config["undistorted_pointcloud"].as<std::string>();

        // subscribers:
        // a. 激光雷达:
        if (lidar_type == Livox) 
        {
            cloud_sub_ptr_ = std::make_shared<LivoxCloudSubscriber>(node_, raw_pointcloud_topic, 100000);
        }else 
        {
            cloud_sub_ptr_ = std::make_shared<StandardCloudSubscriber>(node_, raw_pointcloud_topic, 100000);

        }

        // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(node_, raw_pointcloud_topic, 100000);
        // b. IMU:
        imu_sub_ptr_ = std::make_shared<ImuSubscriber>(node_, imu_raw_data_topic, 1000000);
        
        lidar_to_imu_ptr_ = std::make_shared<TFListener>(node_, lidar_link, car_base_link);

        // publishers:
        // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, undistorted_pointcloud_topic, "/velo_link", 100);

        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(node_, undistorted_pointcloud_topic, global_frame_id, 100);
        imu_pub_ptr_ = std::make_shared<IMUPublisher>(node_, imu_synced_data_topic, imu_link, 100);
        

        // motion compensation for lidar measurement:
        // livox_dedistortion_ptr_ = std::make_shared<LivoxDedistortion>(node_,"/livox/lidar","/livox/imu");
        // distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
        // LivoxDedistortion_thread_start();
        green_info("DataPretreatFlow 初始化完成");
}

bool DataPretreatFlow::Run() 
{

    if (!ReadData())
        return false;

    if (!InitCalibration()) 
        return false;

        // green_info("InitCalibration 初始化完成");

    while(HasData()) 
    {   
        if (!ValidData())
            continue;

        // white_info("livox_ros_driver2::msg::CustomMsg current_cloud_data_.cloud_ptr size: " +
                    // std::to_string(current_cloud_data_.cloud_ptr->size()));


        // lidar_dedistortion();
        if(lidar_cloud_sift())
            PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() 
{
    static std::deque<ImuData> unsynced_imu_;

    // 从缓冲区获取测量值：
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);

    if (cloud_data_buff_.size() == 0)
        return false;

    // 用雷达观测值的时间戳作为参考：
    double cloud_time = cloud_data_buff_.front().time_stamp_;
    builtin_interfaces::msg::Time cloud_ros2_time = cloud_data_buff_.front().ros2_time;
    // sync IMU with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    bool valid_imu = ImuData::SyncData(unsynced_imu_, imu_data_buff_, cloud_ros2_time , cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    static bool sensor_inited = false;
    if (!sensor_inited) 
    {
        if (!valid_imu ) 
        {
            cloud_data_buff_.pop_front();
                return false;
        }
        sensor_inited = true;
    }

    return true;
}

//标定初始化 
bool DataPretreatFlow::InitCalibration()
{
    static bool calibration_received = false;
    if (!calibration_received) 
    {   
        // lidar_to_imu_ 默认是 0,0,0即雷达和imu之间不需要标定
        if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_)) 
        {
            calibration_received = true;
        }
    }

    return calibration_received;
}


bool DataPretreatFlow::HasData() 
{
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
        

    return true;
}

bool DataPretreatFlow::ValidData() 
{   
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time_stamp_ - current_imu_data_.time_stamp_;
    //
    // 该0.05s检查假设激光雷达的频率为10Hz,注意不同的雷达帧率，这个数据应有不同：
    if (diff_imu_time < -0.025  ) 
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.025) 
    {
        imu_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::lidar_dedistortion() 
{
    // c. motion compensation for lidar measurements:
    // current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    // distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);

    // 把里程计端给的线速度，imu给的角速度，扔去做机械式雷达畸变矫正,不确定，先放
    // distortion_adjust_ptr_->SetMotionInfo(0.1, current_imu_data_,current_fused_odom_data_.vel.v);

    // distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr_, current_cloud_data_.cloud_ptr_);
    // livox_dedistortion_ptr_->ProcessOnce();
    return true;
}

/**
     * @brief 这里尝试筛掉扫到车体上的点，剔除感知给到的动态障碍物点
     **/
bool DataPretreatFlow::lidar_cloud_sift()
{

    //哨兵mid-360遮挡部分按距离滤除
    //处理离雷达30cm的点云以及z方向上小于 +10cm的点云

    pcl::PassThrough<pcl::PointXYZINormal> pass;
    pass.setInputCloud (current_cloud_data_.cloud_ptr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (true);
    pass.setFilterLimits (-0.3, 0.3);
    pass.filter (*(current_cloud_data_.cloud_ptr));

    pass.setFilterFieldName ("y");
    pass.setFilterLimitsNegative (true);
    pass.setFilterLimits (0.3, 0.3);
    pass.filter (*(current_cloud_data_.cloud_ptr));

    pass.setFilterFieldName ("z");
    pass.setFilterLimitsNegative (true);
    pass.setFilterLimits (-10, 1.414);
    pass.filter (*(current_cloud_data_.cloud_ptr));

    //处理离雷达30cm的点云以及z方向上小于 +18cm的点云
                
    return true;
}

bool DataPretreatFlow::PublishData() 
{   
    
    // white_info("PublishData current_cloud_data_.cloud_ptr size: "+ std::to_string(current_cloud_data_.cloud_ptr->size()) );
    // take lidar measurement time as synced timestamp:
    const double &timestamp_synced = current_cloud_data_.time_stamp_;
    // const builtin_interfaces::msg::Time &ros2_timestamp_synced = current_cloud_data_.ros2_time;
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.ros2_time);
    imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.ros2_time);
    // green_info("DataPretreatFlow::PublishData done");

    
    return true;
}


void DataPretreatFlow::LivoxDedistortion_thread_start()
{
    std::thread work_thread(LivoxDedistortion_thread_working,this);
	work_thread.detach();
}
void DataPretreatFlow::LivoxDedistortion_thread_working(DataPretreatFlow* ptr_)
{
    ptr_->livox_dedistortion_ptr_->ProcessLoop(ptr_->ros2_node);
}

}