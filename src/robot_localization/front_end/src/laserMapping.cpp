#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <pcl/io/pcd_io.h>

#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "parameters.h"
#include "Estimator.h"
#include "timestamp_transform.hpp"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "localization_tools/load_pcd_file.hpp"
#include "localization_tools/color_terminal.hpp"
#include "localization_models/measure_compensation.hpp"
#include "localization_models/registration_score.hpp"
#include "localization_tools/tic_toc.hpp"
#include <yaml-cpp/yaml.h>
#include <localization_interface/msg/lam2_nav.hpp>

#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());

// Time Log Variables
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool   lidar_pushed = false, flg_reset = false, flg_exit = false;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>               time_buffer;
deque<sensor_msgs::msg::Imu::ConstPtr> imu_deque;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped odomAftMapped_2;//备选的定位广播的第二种消息格式，主要是空间占用小

geometry_msgs::msg::PoseStamped msg_body_pose;
int relocation_set_from_outside_flag=0;  //外部重定位标识，0为默认不使能，1为要求执行，2为执行完成
nav_msgs::msg::Odometry relocation_pose_from_outside;

double average_distance_score = 0.0;    // 平均距离得分
double overlap_score = 0.0;             // 重叠度得分,只在计算由 重叠点/有效距离点 时进行更改，用于评分以更新地图/重定位和发布定位置信度
int overlap_points_num = 0;             // 重叠点数
int effective_score_points_num = 0 ;    // 可获取距离的点数
bool LAM_trustable = false;             // LAM定位是否可信
double aver_dis_scores;
double aver_olap_scores;
geometry_msgs::msg::Pose last_true_LAM_pose; // 记录上一帧真值位姿，用于重定位

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("a_logger_SigHandle"),"catch sig %d", sig);


    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang;
    if (!use_imu_as_input)
    {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    }
    else
    {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }
    
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input)
    {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2)); // Bias_a  
    }
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

int points_cache_size = 0;

void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history); // 返回被剔除的点
    // ikdtree.acquire_removed_points()函数的主要作用是从iKD-Tree中提取已删除的点。
    // 在处理动态点云数据时，我们需要根据新的观测结果更新地图。这涉及到将不再可见或被遮挡的点从地图中移除。acquire_removed_points()函数在这个过程中有以下用途：
    // 提取被移除的点：从iKD-Tree中获取已被删除的点，这些点可能是因为不再可见、被遮挡或者地图更新而被移除。
    // 用于地图更新：在动态环境中，我们需要实时更新地图以适应环境变化。acquire_removed_points()函数提供了获取被移除点的方法，以便在地图更新过程中将这些点从地图中剔除。
    // 用于回环检测和优化：在某些情况下，我们需要获取被移除的点以进行回环检测和优化。这些点可以提供额外的信息，有助于提高回环检测的准确性和优化结果。

    points_cache_size = points_history.size();
}


// 动态调整地图区域，防止地图过大而内存溢出，类似LOAM中提取局部地图的方法
BoxPointType LocalMap_Points; // ikd-tree中,局部地图的包围盒角点
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    if (use_imu_as_input)
    {
        // pos_LiD = state_in.pos + state_in.rot * Lidar_T_wrt_IMU;
        // pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;


    }
    else
    {
        // pos_LiD = state_out.pos + state_out.rot * Lidar_T_wrt_IMU;
        // pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;

    }
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if(cub_needrm.size() > 0) 
        int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;      // 更新雷达扫描计数 scan_count
    double preprocess_start_time = omp_get_wtime();
    // if (msg->header.stamp.toSec() < last_timestamp_lidar)    //检查传入消息的时间戳是否小于上一个雷达时间戳，如果是则清除缓冲区并退出
    if ( time_to_double(msg->header.stamp) < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("a_logger_standard_pcl_cbk"), "lidar loop back, clear buffer.");
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    // last_timestamp_lidar = msg->header.stamp.toSec();
    last_timestamp_lidar = time_to_double(msg->header.stamp);   // 更新上一个雷达时间戳


    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    double time_div = time_to_double(msg->header.stamp);
    p_pre->process(msg, ptr);
    if (cut_frame)      // 如果 cut_frame 为真，将按时间间隔将点云分割为多个子点云并添加到缓冲区
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + time_to_double(msg->header.stamp) - time_div > cut_frame_time_interval)
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)     // 如果 con_frame 为真，将根据指定的帧数将连续点云连接在一起并添加到缓冲区
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else        // 如果 cut_frame 和 con_frame 都不为真，将整个点云添加到缓冲区
    { 
        lidar_buffer.emplace_back(ptr);
        // time_buffer.emplace_back(msg->header.stamp.toSec());
        time_buffer.emplace_back(time_to_double(msg->header.stamp));

    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;//计算预处理所需的时间并存储在 s_plot11 数组中
    mtx_buffer.unlock();
    sig_buffer.notify_all();    //使用 sig_buffer.notify_all() 通知所有等待该缓冲区的线程
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (time_to_double(msg->header.stamp) < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("a_logger_livox_pcl_cbk"), "lidar loop back, clear buffer.");


        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = time_to_double(msg->header.stamp);    
    TicToc exam11;
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);   // ptr 为预处理后的点云

    PointCloudXYZI::Ptr ptr_filtered(new PointCloudXYZI(ptr->size(), 1)); // Create a new point cloud for the filtered points
    std::vector<PointType> new_ptr;
    int ptr_size = ptr->size();
    
    
    // std::cout << "ptr size:" << ptr_size << std::endl;
// #pragma omp parallel for

    // for(int i = 0; i < ptr_size; i++)
    // {       
    //         pointBodyToWorld(&ptr->points[i], &ptr_filtered->points[i]);
    //         if(ptr_filtered->points[i].z < 1.42) {
    //             ptr->erase(ptr->points.begin() + i);
    //             ptr_filtered->erase(ptr_filtered->points.begin() + i);
    //             --i; // Since we just erased a point, we need to decrement the index
    //             --ptr_size; // Update the size
    //         }
            
    // }
    for(int i = 0; i < ptr_size; i++) 
    {       
        pointBodyToWorld(&ptr->points[i], &ptr_filtered->points[i]);
        if(ptr_filtered->points[i].z >= 0.4) 
            new_ptr.push_back(ptr->points[i]);
        
    }
    ptr->points.assign(new_ptr.begin(), new_ptr.end());

    // std::cout << "ptr_size size:" << ptr->size() << std::endl;
    // ptr = ptr_filtered;
    // std::cout << "mount correct and filter time:" << exam11.toc()/1000.0 <<"s"<< std::endl;

    // // 这一部分想要利用  p_imu->linear_acc_compensation_eigen的补偿矩阵
    // // 将点云中的点进行补偿，使点云的整体坐标矫正到重力水平面上
    // int size = ptr->size();
    // for (int i = 0; i < size; i++)
    // {
    //     compensateLidarMeasurement(&ptr->points[i],&ptr->points[i],ImuMount_compensate_matrix3d);
    // }
    // // 调用 p_imu->linear_acc_compensation_eigen 对点云进行补偿
    double time_div = time_to_double(msg->header.stamp);
    if (cut_frame)
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + time_to_double(msg->header.stamp) - time_div > cut_frame_time_interval)
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_div_i = *ptr_div;
                // cout << "ptr div i num:" << ptr_div_i->size() << endl;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(time_to_double(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));
    compensateImuMeasurement(*msg,ImuMount_compensate_matrix3d);

    // msg->header.stamp = double_to_time( time_to_double(msg_in->header.stamp)- time_diff_lidar_to_imu );
    msg->header.stamp = double_to_time((time_to_double(msg_in->header.stamp) - time_lag_imu_to_lidar));

    // double timestamp = msg->header.stamp.toSec();
    double timestamp = time_to_double(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_ERROR(rclcpp::get_logger("a_logger_imu_cbk"), "imu loop back, clear deque.");

        // imu_deque.shrink_to_fit();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void relocalization_cbk(const nav_msgs::msg::Odometry::ConstPtr &odom_msg)
{   
    
    if(relocation_set_from_outside_flag==0)
    {
        static nav_msgs::msg::Odometry begining_relocalization = *odom_msg;
        static int check_flag = 0;
        if(
            abs(begining_relocalization.pose.pose.position.x-odom_msg->pose.pose.position.x)<0.15
          &&  abs(begining_relocalization.pose.pose.position.y-odom_msg->pose.pose.position.y)<0.15
          &&  abs(begining_relocalization.pose.pose.position.z-odom_msg->pose.pose.position.z)<0.15
          &&  abs(begining_relocalization.pose.pose.orientation.x-odom_msg->pose.pose.orientation.x)<0.08
          &&  abs(begining_relocalization.pose.pose.orientation.y-odom_msg->pose.pose.orientation.y)<0.08
          &&  abs(begining_relocalization.pose.pose.orientation.z-odom_msg->pose.pose.orientation.z)<0.08
          &&  abs(begining_relocalization.pose.pose.orientation.w-odom_msg->pose.pose.orientation.w)<0.08
          )
            check_flag++;
        else
            check_flag=0,
            begining_relocalization = *odom_msg;

          if(check_flag>9)
            relocation_pose_from_outside = begining_relocalization,
            relocation_set_from_outside_flag = 1, //可以使用 relocation_pose_from_outside 进行初始重定位
            std::cout<<"确定重定位：\n"<<"x "<<relocation_pose_from_outside.pose.pose.position.x<<
            "  y "<<relocation_pose_from_outside.pose.pose.position.y<<
            "  z "<<relocation_pose_from_outside.pose.pose.position.z<<"\n";



    }
        
}

/**
 * @brief 读取并同步传感器信息
 * @param 
 **/
bool sync_packages(MeasureGroup &meas)
{
    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if(meas.lidar->points.size() < 1) 
            {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points)
        {
            if (pt.curvature > end_time)
            {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        
        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_)
    {
        // double imu_time = imu_deque.front()->header.stamp.toSec();
        double imu_time = time_to_double(imu_deque.front()->header.stamp);

        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            // imu_time = imu_deque.front()->header.stamp.toSec(); 
            imu_time = time_to_double(imu_deque.front()->header.stamp); 

            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }
    else if(!init_map)
    {
        // double imu_time = imu_deque.front()->header.stamp.toSec();
        double imu_time = time_to_double(imu_deque.front()->header.stamp);

        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = time_to_double(imu_deque.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;

/**
 * @brief 建图加入点云信息
 * @param 
 **/
void map_incremental()
{
    PointVector PointToAdd;  // 需要在ikd-tree地图中新增的点云
    PointVector PointNoNeedDownsample;  //加入ikd-tree时，不需要降采样的点云
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    
    for(int i = 0; i < feats_down_size; i++)
    {
        /* No points found within the given threshold of nearest search*/
        // if (Nearest_Points[i].empty())
        // {
        //     PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        //     continue;          
        // }      
        /* decide if need add to map */
        
        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;

            PointType downsample_result, mid_point; 
            // 体素滤波器长度：filter_size_map_min，这个参数在fast-lio中设为0.1,在point-lio就比较大
            // mid_point即为该特征点所属的栅格的中心点坐标
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            /* If the nearest points is definitely outside the downsample box */
            if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min){
                // 若三个方向距离都大于地图栅格半轴长，无需降采样
                // 如果最近的点 points_near[0]在体素格子之外，那么就直接将该特征点添加到PointNoNeedDownsample向量，并跳过当前循环
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                continue;
            }
            /* Check if there is a point already in the downsample box and closer to the center point */
            // 二范数：dist
            float dist  = calc_dist<float>(feats_down_world->points[i],mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i ++)
            {
                // if (points_near.size() < NUM_MATCH_POINTS) break;
                /* Those points which are outside the downsample box should not be considered. */
                // if (fabs(points_near[readd_i].x - mid_point.x) > 0.5 * filter_size_map_min || fabs(points_near[readd_i].y - mid_point.y) > 0.5 * filter_size_map_min || fabs(points_near[readd_i].z - mid_point.z) > 0.5 * filter_size_map_min) {
                //     continue;                    
                // }
                // if (calc_dist<float>(points_near[readd_i], mid_point) < dist)
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) 
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
        }
        else  // 初始点
        {
            // PointToAdd.emplace_back(feats_down_world->points[i]);
            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }


    // ikdtree.Add_Points(PointToAdd, true);
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);// ikd-tree维护的点云地图加入

}

void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr   laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);// 前序遍历和后序遍历都有，将一个树的所有节点展平，按顺序放到一个容器内
                
    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);
        
    laserCloudmsg.header.stamp = double_to_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "map";
    pubLaserCloudFullRes->publish(laserCloudmsg);

}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();
        int size_2 = feats_undistort->points.size();    // 来自fast-lio2 的启示

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size+size_2, 1));
        
        for (int i = 0; i < size; i++)
        {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // 
            // }
        }
        for (int i = size; i < (size+size_2); i++)
        {
            pointBodyToWorld(&feats_undistort->points[i-size], \
                            &laserCloudWorld->points[i]);    // 来自fast-lio2 的启示，增加建图点云数
                            
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp = double_to_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "map";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        int size_2 = feats_undistort->points.size();        // 来自fast-lio2 的启示
        //建图时 feats_down_world 的点云差不多 feats_undistort的 六分之一
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size+size_2, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        for (int i = size; i < (size+size_2); i++)
        {
            pointBodyToWorld(&feats_undistort->points[i-size], \
                            &laserCloudWorld->points[i]);    // 来自fast-lio2 的启示

            // laserCloudWorld->points[i].x = feats_undistort->points[i-size].x;
            // laserCloudWorld->points[i].y = feats_undistort->points[i-size].y;
            // laserCloudWorld->points[i].z = feats_undistort->points[i-size].z;
            // laserCloudWorld->points[i].intensity = feats_undistort->points[i-size].intensity;
        }


        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "slam_pcd_maps/slam_scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

// 匹配时输入的点云，将那一段也写入储存
void store_world_for_matching()
{
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = init_feats_world->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = init_feats_world->points[i].x;
            laserCloudWorld->points[i].y = init_feats_world->points[i].y;
            laserCloudWorld->points[i].z = init_feats_world->points[i].z;
            laserCloudWorld->points[i].intensity = init_feats_world->points[i].intensity;
        }


        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "slam_pcd_maps/slam_scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}
void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = double_to_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "map";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        out.orientation.x = kf_output.x_.rot.coeffs()[0];
        out.orientation.y = kf_output.x_.rot.coeffs()[1];
        out.orientation.z = kf_output.x_.rot.coeffs()[2];
        out.orientation.w = kf_output.x_.rot.coeffs()[3];
    }
    else
    {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        out.orientation.x = kf_input.x_.rot.coeffs()[0];
        out.orientation.y = kf_input.x_.rot.coeffs()[1];
        out.orientation.z = kf_input.x_.rot.coeffs()[2];
        out.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

void set_init_poses(nav_msgs::msg::Odometry init_pose_to_set)
{
    // if (!use_imu_as_input)
    // {
        kf_output.x_.pos(0) = init_pose_to_set.pose.pose.position.x;
        kf_output.x_.pos(1) = init_pose_to_set.pose.pose.position.y;
        kf_output.x_.pos(2) = init_pose_to_set.pose.pose.position.z;
        kf_output.x_.rot.coeffs()[0] = init_pose_to_set.pose.pose.orientation.x;
        kf_output.x_.rot.coeffs()[1] = init_pose_to_set.pose.pose.orientation.y;
        kf_output.x_.rot.coeffs()[2] = init_pose_to_set.pose.pose.orientation.z;
        kf_output.x_.rot.coeffs()[3] = init_pose_to_set.pose.pose.orientation.w;
        
    // }
    // else
    // {
        kf_input.x_.pos(0) = init_pose_to_set.pose.pose.position.x;
        kf_input.x_.pos(1) = init_pose_to_set.pose.pose.position.y;
        kf_input.x_.pos(2) = init_pose_to_set.pose.pose.position.z;
        kf_input.x_.rot.coeffs()[0] = init_pose_to_set.pose.pose.orientation.x;
        kf_input.x_.rot.coeffs()[1] = init_pose_to_set.pose.pose.orientation.y;
        kf_input.x_.rot.coeffs()[2] = init_pose_to_set.pose.pose.orientation.z;
        kf_input.x_.rot.coeffs()[3] = init_pose_to_set.pose.pose.orientation.w;
    // }
}

void relocalization_set(const geometry_msgs::msg::Pose &the_pose_to_set)
{
    // if (!use_imu_as_input)
    // {
        kf_output.x_.pos(0) = the_pose_to_set.position.x;
        kf_output.x_.pos(1) = the_pose_to_set.position.y;
        kf_output.x_.pos(2) = the_pose_to_set.position.z;
        kf_output.x_.rot.coeffs()[0] = the_pose_to_set.orientation.x;
        kf_output.x_.rot.coeffs()[1] = the_pose_to_set.orientation.y;
        kf_output.x_.rot.coeffs()[2] = the_pose_to_set.orientation.z;
        kf_output.x_.rot.coeffs()[3] = the_pose_to_set.orientation.w;
        
    // }
    // else
    // {
        kf_input.x_.pos(0) = the_pose_to_set.position.x;
        kf_input.x_.pos(1) = the_pose_to_set.position.y;
        kf_input.x_.pos(2) = the_pose_to_set.position.z;
        kf_input.x_.rot.coeffs()[0] = the_pose_to_set.orientation.x;
        kf_input.x_.rot.coeffs()[1] = the_pose_to_set.orientation.y;
        kf_input.x_.rot.coeffs()[2] = the_pose_to_set.orientation.z;
        kf_input.x_.rot.coeffs()[3] = the_pose_to_set.orientation.w;
    // }
}

void publish_LAM(   const rclcpp::Publisher<localization_interface::msg::LAM2Nav>::SharedPtr & pubLAM2Nav,
                    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &pubOdomForDebug,
                    std::shared_ptr<rclcpp::Node>& node_
                )
{   
    static double last_update_time = time_to_double(node_->now() );
    if (time_to_double(node_->now() ) <= (last_update_time+0.0032))
    {
        return;
    }

    localization_interface::msg::LAM2Nav LAM2Nav_msg;
    LAM2Nav_msg.header.frame_id = "map";
    odomAftMapped_2.header.frame_id = "map";

    if (publish_odometry_without_downsample)
    {
        LAM2Nav_msg.header.stamp = double_to_time(time_current);
        odomAftMapped_2.header.stamp = double_to_time(time_current);

    }
    else
    {
        LAM2Nav_msg.header.stamp = double_to_time(lidar_end_time);
        odomAftMapped_2.header.stamp = double_to_time(time_current);
    }
    set_posestamp(LAM2Nav_msg.pose);
    odomAftMapped_2.pose = LAM2Nav_msg.pose;
    
    // 广播速度与线速度消息
    if (!use_imu_as_input)
    {   
        LAM2Nav_msg.linear_vel.x = kf_output.x_.vel.x();
        LAM2Nav_msg.linear_vel.y = kf_output.x_.vel.y();
        LAM2Nav_msg.linear_vel.z = kf_output.x_.vel.z();
        LAM2Nav_msg.linear_acc.x = kf_output.x_.acc.x();
        LAM2Nav_msg.linear_acc.y = kf_output.x_.acc.y();
        LAM2Nav_msg.linear_acc.z = kf_output.x_.acc.z();

        // 关于速度方向不在水平面的问题，已经用补偿矩阵在雷达和IMU的回调已完成
        
    }
    else
    {   
        LAM2Nav_msg.linear_vel.x = kf_input.x_.vel.x();
        LAM2Nav_msg.linear_vel.y = kf_input.x_.vel.y();
        LAM2Nav_msg.linear_vel.z = kf_input.x_.vel.z();
        
        LAM2Nav_msg.linear_acc.x = imu_next.linear_acceleration.x*9.80;
        LAM2Nav_msg.linear_acc.y = imu_next.linear_acceleration.y*9.80;
        LAM2Nav_msg.linear_acc.z = imu_next.linear_acceleration.z*9.80;

    }
    LAM2Nav_msg.reliability = LAM_trustable;
    if(LAM_trustable)
        last_true_LAM_pose = LAM2Nav_msg.pose;
    pubLAM2Nav->publish(LAM2Nav_msg);
    pubOdomForDebug->publish(odomAftMapped_2);   //tdt_localization_cheng

    static tf2_ros::TransformBroadcaster br(node_);
    tf2::Transform transform;
    tf2::Quaternion q;
    // 发布变换
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg.transform.translation.x = odomAftMapped_2.pose.position.x;
    transform_stamped_msg.transform.translation.y = odomAftMapped_2.pose.position.y;
    transform_stamped_msg.transform.translation.z = odomAftMapped_2.pose.position.z;
    transform_stamped_msg.transform.rotation.w = odomAftMapped_2.pose.orientation.w;
    transform_stamped_msg.transform.rotation.x = odomAftMapped_2.pose.orientation.x;
    transform_stamped_msg.transform.rotation.y = odomAftMapped_2.pose.orientation.y;
    transform_stamped_msg.transform.rotation.z = odomAftMapped_2.pose.orientation.z;
    transform_stamped_msg.header.stamp = odomAftMapped_2.header.stamp;
    transform_stamped_msg.header.frame_id = "map";
    transform_stamped_msg.child_frame_id = "aft_mapped_pose";
    br.sendTransform(transform_stamped_msg);
    last_update_time = time_to_double(node_->now() );
}


void publish_odometry(
                        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pubOdomAftMapped,
                        std::shared_ptr<rclcpp::Node>& node_,
                        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pubOdomAftMapped_2,
                        const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & pubLinearVelAndAcc
                     )
{   
    static double last_update_time = time_to_double(node_->now() );

    if (time_to_double(node_->now() ) > (last_update_time+0.0032))
    {
        odomAftMapped_2.header.frame_id = "map";
        geometry_msgs::msg::TwistStamped LinearVelAndAcc_msg;

        if (publish_odometry_without_downsample)
        {
            odomAftMapped_2.header.stamp = double_to_time(time_current);
            LinearVelAndAcc_msg.header.stamp = double_to_time(time_current);

        }
        else
        {
            odomAftMapped_2.header.stamp = double_to_time(lidar_end_time);
            LinearVelAndAcc_msg.header.stamp = double_to_time(lidar_end_time);

        }

        set_posestamp(odomAftMapped_2.pose);
        // 广播速度与线速度消息
        if (!use_imu_as_input)
        {
            LinearVelAndAcc_msg.twist.linear.x = kf_output.x_.vel.x();
            LinearVelAndAcc_msg.twist.linear.y = kf_output.x_.vel.y();
            LinearVelAndAcc_msg.twist.linear.z = kf_output.x_.vel.z();

            // LinearVelAndAcc_msg.twist.linear.x = state_out.vel.x();
            // LinearVelAndAcc_msg.twist.linear.y = state_out.vel.y();
            // LinearVelAndAcc_msg.twist.linear.z = state_out.vel.z();

            double linear_acc_x,linear_acc_y,linear_acc_z;
            linear_acc_x = kf_output.x_.acc.x();
            linear_acc_y = kf_output.x_.acc.y();
            linear_acc_z = kf_output.x_.acc.z();

            // linear_acc_x = state_out.acc.x();
            // linear_acc_y = state_out.acc.y();
            // linear_acc_z = state_out.acc.z();
            // 应用前面算出的补偿矩阵，左乘加速度，矫正 linear_acc_
            compensateImuMeasurement(   
                                        linear_acc_x,
                                        linear_acc_y,
                                        linear_acc_z,
                                        p_imu->linear_acc_compensation_eigen
                                    );
            LinearVelAndAcc_msg.twist.angular.x = linear_acc_x;
            LinearVelAndAcc_msg.twist.angular.y = linear_acc_y;
            LinearVelAndAcc_msg.twist.angular.z = linear_acc_z;
            // 应用前面算出的补偿矩阵，左乘加速度，矫正 linear_acc_

            // LinearVelAndAcc_msg.twist.angular.x = imu_next.linear_acceleration.x*9.81;
            // LinearVelAndAcc_msg.twist.angular.y = imu_next.linear_acceleration.y*9.81;
            // LinearVelAndAcc_msg.twist.angular.z = imu_next.linear_acceleration.z*9.81;


        }
        else
        {   
            LinearVelAndAcc_msg.twist.linear.x = kf_input.x_.vel.x();
            LinearVelAndAcc_msg.twist.linear.y = kf_input.x_.vel.y();
            LinearVelAndAcc_msg.twist.linear.z = kf_input.x_.vel.z();
            // LinearVelAndAcc_msg.twist.angular.x = kf_output.x_.acc.x();
            // LinearVelAndAcc_msg.twist.angular.y = kf_output.x_.acc.y();
            // LinearVelAndAcc_msg.twist.angular.z = kf_output.x_.acc.z();
            double linear_acc_x,linear_acc_y,linear_acc_z;
            linear_acc_x = imu_next.linear_acceleration.x*9.81;
            linear_acc_y = imu_next.linear_acceleration.y*9.81;
            linear_acc_z = imu_next.linear_acceleration.z*9.81;
            // 应用前面算出的补偿矩阵，左乘加速度，矫正 linear_acc_
            compensateImuMeasurement(   
                                        linear_acc_x,
                                        linear_acc_y,
                                        linear_acc_z,
                                        p_imu->linear_acc_compensation_eigen
                                    );
            LinearVelAndAcc_msg.twist.angular.x = linear_acc_x;
            LinearVelAndAcc_msg.twist.angular.y = linear_acc_y;
            LinearVelAndAcc_msg.twist.angular.z = linear_acc_z;
            // 应用前面算出的补偿矩阵，左乘加速度，矫正 linear_acc_


        }
        // 广播速度与线速度消息

        
        pubOdomAftMapped->publish(odomAftMapped_2);   //tdt_localization_cheng
        pubOdomAftMapped_2->publish(odomAftMapped_2);//tdt_localization_001

        pubLinearVelAndAcc->publish(LinearVelAndAcc_msg);
        
        static tf2_ros::TransformBroadcaster br(node_);
        tf2::Transform transform;
        tf2::Quaternion q;

        // static tf::TransformBroadcaster br;
        // tf::Transform                   transform;
        // tf::Quaternion                  q;
        // transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
        //                                 odomAftMapped.pose.pose.position.y, \
        //                                 odomAftMapped.pose.pose.position.z));
        // q.setW(odomAftMapped.pose.pose.orientation.w);
        // q.setX(odomAftMapped.pose.pose.orientation.x);
        // q.setY(odomAftMapped.pose.pose.orientation.y);
        // q.setZ(odomAftMapped.pose.pose.orientation.z);
        // transform.setRotation( q );
        // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "map", "aft_mapped" ) );


        // 发布变换
        geometry_msgs::msg::TransformStamped transform_stamped_msg;
        transform_stamped_msg.transform.translation.x = odomAftMapped_2.pose.position.x;
        transform_stamped_msg.transform.translation.y = odomAftMapped_2.pose.position.y;
        transform_stamped_msg.transform.translation.z = odomAftMapped_2.pose.position.z;
        transform_stamped_msg.transform.rotation.w = odomAftMapped_2.pose.orientation.w;
        transform_stamped_msg.transform.rotation.x = odomAftMapped_2.pose.orientation.x;
        transform_stamped_msg.transform.rotation.y = odomAftMapped_2.pose.orientation.y;
        transform_stamped_msg.transform.rotation.z = odomAftMapped_2.pose.orientation.z;
        transform_stamped_msg.header.stamp = odomAftMapped_2.header.stamp;
        transform_stamped_msg.header.frame_id = "map";
        transform_stamped_msg.child_frame_id = "aft_mapped_pose";
        br.sendTransform(transform_stamped_msg);
        last_update_time = time_to_double(node_->now() );
    }
    
}
void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    // msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.stamp = double_to_time(lidar_end_time);

    msg_body_pose.header.frame_id = "map";
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}        


class WaitingLocalizationStart : public rclcpp::Node
{
public:
  WaitingLocalizationStart() : Node("to_start_localization_end")
  {
    server_ = create_service<std_srvs::srv::SetBool>("/localization_start", std::bind(&WaitingLocalizationStart::callback, this, std::placeholders::_1, std::placeholders::_2));
    request_received_=false;
    
  }

public:
  void callback(const std_srvs::srv::SetBool::Request::SharedPtr request, const std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    // 处理服务请求，这里的示例是直接将服务请求的data返回
    response->success = true;
    response->message = "localization is to start";
    
    // 服务处理完毕后发送信号，通知等待的线程可以继续执行
    // std::lock_guard<std::mutex> lock(mutex_);
    request_received_ = true;
    // waiting_start_signal.notify_one();
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
//   std::mutex mutex_;
//   std::condition_variable waiting_start_signal;
  bool request_received_ = false;
};

int main(int argc, char** argv)
{   
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    auto to_start_localization_node = std::make_shared<WaitingLocalizationStart>();
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("laserMapping_end");
    readParameters(ros2_node);

    if(ikdtree.Root_Node == nullptr) //
    // if(feats_down_size > 5)
    {   // 设置ikd tree的降采样参数
        ikdtree.set_downsample_param(filter_size_map_min);
    }
    if(pcd_map_matching_en)
    {   
        string map_path_one(string(string(ROOT_DIR) + matching_pcd_map_path));
        green_info("图加入优化模式，开始加载地图:"+map_path_one);
        
        TicToc exam;
        pcl::io::loadPCDFile(map_path_one, *feats_down_world);
        green_info("加载地图耗时 "+std::to_string(exam.toc()/1000.00)+" s");

        // 组织ikd tree
        for (size_t i = 0; i < feats_down_world->size(); i++) 
        {
            init_feats_world->points.emplace_back(feats_down_world->points[i]);
        }
                
        TicToc exam2;
        ikdtree.Build(init_feats_world->points);

        green_info("构建ikd-tree 地图耗时 "+std::to_string(exam2.toc()/1000.00)+" s");

        // ikdtree.Build(feats_down_world->points); 
        std::cout<<"地图 load over, ikdtree Build\n";
    }
    
    ColorTerminal::cyan("localization online,waiting for signal\
                            or keyboard:\nros2 service call /localization_start std_srvs/srv/SetBool data:\\ true");

    last_true_LAM_pose.position.x = 0.0;
    last_true_LAM_pose.position.y = 0.0;
    last_true_LAM_pose.position.z = 0.0;
    last_true_LAM_pose.orientation.x = 0.0;
    last_true_LAM_pose.orientation.y = 0.0;
    last_true_LAM_pose.orientation.z = 0.0;
    last_true_LAM_pose.orientation.w = 1.0;
    
    
    path.header.stamp    = double_to_time(lidar_end_time);
    path.header.frame_id ="map";

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;
    
    /*** initialize variables ***/
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
        else
        {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init = MD(24,24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30,30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;



//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    

while (!to_start_localization_node->request_received_) 
{
    rclcpp::spin_some(to_start_localization_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
}
RCLCPP_INFO(to_start_localization_node->get_logger(), "收到启动信号, localization start");
to_start_localization_node.reset();    // 销毁 to_start_localization_node

// 通信subscribers and publishers ///////////////////////////////////
    rclcpp::SubscriptionOptions sub_options;
    rclcpp::PublisherOptions pub_options;
    rclcpp::QoS qos_best_effort(0);     // 选择reliable reliability和transient_local durability配置可以在通信中断后保留未传递的消息，并在重新连接后重新发送
    qos_best_effort.best_effort();  // 设置为尽力而为模式，即使消息丢失也不会重发
    sub_options.use_default_callbacks = false;
    sub_options.event_callbacks.incompatible_qos_callback = [](rclcpp::QOSOfferedIncompatibleQoSInfo & event) {
        std::cout << "Custom Incompatible Callback!" << std::endl;
    };
    pub_options.use_default_callbacks = false;
    pub_options.event_callbacks.incompatible_qos_callback = [](rclcpp::QOSOfferedIncompatibleQoSInfo & event) {
        std::cout << "Custom Incompatible Callback!" << std::endl;
    };
    rclcpp::SubscriptionBase::SharedPtr sub_pcl;
    if (p_pre->lidar_type == AVIA) 
    {
        sub_pcl = ros2_node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic,
            // 200000,
            qos_best_effort, 
            [](const livox_ros_driver2::msg::CustomMsg::ConstPtr& msg) 
            {
                // Process the received data using the livox_pcl_cbk function
                livox_pcl_cbk(msg);
            },
            sub_options
            );
    } else 
    {
        sub_pcl = ros2_node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, 
            // 200000, 
            qos_best_effort, 
            [](const sensor_msgs::msg::PointCloud2::ConstPtr& msg) 
            {
                // Process the received data using the standard_pcl_cbk function
                standard_pcl_cbk(msg);
            },
            sub_options
            );
    }

    

    auto sub_imu = ros2_node->
            create_subscription<sensor_msgs::msg::Imu>(
                                                        imu_topic,
                                                        // 2000,  // 保留原始的队列长度为200000
                                                        qos_best_effort,  // 使用可靠传输的QoS配置
                                                        std::bind(&imu_cbk,  std::placeholders::_1),
                                                        sub_options
                                                      );
    auto relocalization_odom_sub = ros2_node->
            create_subscription<nav_msgs::msg::Odometry>(
                                                        "map_matching_odom",
                                                        // 10,
                                                        qos_best_effort,  // 使用可靠传输的QoS配置
                                                        std::bind(&relocalization_cbk,  std::placeholders::_1),
                                                        sub_options
                                                      );                                                  
                     

    auto pubLaserCloudFullRes = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered",qos_best_effort,pub_options    // ,100000,
                                                                                          );
    auto pubLaserCloudFullRes_body = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body",qos_best_effort,pub_options//  ,100000
     );
    auto pubLaserCloudEffect = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_effected",qos_best_effort,pub_options//, 100000
        );
    auto pubLaserCloudMap = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/Laser_map",qos_best_effort,pub_options//, 100000
        );

    auto pubOdomForDebug = ros2_node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/tdt_localization_cheng",qos_best_effort,pub_options//, 4000
        );//备选的定位广播的第二种消息格式，主要是空间占用小
    auto pubOdomAftMapped_2 = ros2_node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/tdt_localization_001",qos_best_effort,pub_options//, 4000
        );//备选的定位广播的第二种消息格式，主要是空间占用小
    auto LAM_publisher = ros2_node->create_publisher<localization_interface::msg::LAM2Nav>(
        "/tdt_LAM_001",qos_best_effort,pub_options//, 4000
        );//自定义的定位广播消息

    // auto pubOdomAftMapped = ros2_node->create_publisher<nav_msgs::msg::Odometry>("/tdt_localization_cheng", 4000);//备选的定位广播的第二种消息格式，主要是空间占用小
    // auto pubOdomAftMapped_2 = ros2_node->create_publisher<nav_msgs::msg::Odometry>("/tdt_localization_001", 4000);//备选的定位广播的第二种消息格式，主要是空间占用小
    
    auto pubLinearVelAndAcc = ros2_node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/tdt_localization_linear",qos_best_effort,pub_options//, 4000
        );// 广播线加速度和线速度

    auto pubPath = ros2_node->create_publisher<nav_msgs::msg::Path>(
        "/front_end_path",qos_best_effort,pub_options//, 10000
        );
    auto plane_pub = ros2_node->create_publisher<visualization_msgs::msg::Marker>(
        "/planner_normal",qos_best_effort,pub_options//, 1000
        );
// 通信subscribers and publishers ///////////////////////////////////

    rclcpp::Rate rate(5000);
    bool status = rclcpp::ok();
    while (status)
    {
        if (flg_exit) break;
        rclcpp::spin_some(ros2_node);
        if(sync_packages(Measures)) 
        {
            if (flg_first_scan)     // 第一帧雷达数据
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                // cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset)
            {
                RCLCPP_WARN(ros2_node->get_logger(),"reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0,t1,t2,t3,t4,t5,match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();
            // feats_undistort:运动畸变去除之后的点云数据
            // 对IMU数据进行预处理，其中包含了点云畸变处理 前向传播 反向传播???
            p_imu->Process(Measures, feats_undistort);

            if (feats_undistort->empty() || feats_undistort == NULL)
            {
                continue;
            }

            if(imu_en)
            {
                if (!p_imu->gravity_align_)
                {
                    while (Measures.lidar_beg_time > time_to_double(imu_next.header.stamp) )
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        // imu_deque.pop();
                    }
                    // state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    // state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    // state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    // state_out.acc *= -1;

                    state_in.gravity =  -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
                    state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
                    state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                    if (gravity_align)
                    {   
                        std::cout<<"重力进行校准\n";

                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_in.gravity = state_out.gravity = p_imu->gravity_;
                        state_in.rot = state_out.rot = rot_init;
                        state_in.rot.normalize();
                        state_out.rot.normalize();
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    kf_input.change_x(state_in);
                    kf_output.change_x(state_out);
                }
            }
            else
            {
                if (!p_imu->gravity_align_)
                {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
            }
            /*** Segment the map in lidar FOV ***/
            // 动态调整局部地图
            lasermap_fov_segment();
            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            if(space_down_sample)
            {   // 将输入点云 feats_undistort 设置为下采样操作的输入点云。
                downSizeFilterSurf.setInputCloud(feats_undistort);
                // 对输入点云进行下采样操作，并将下采样后的结果存储在 feats_down_body 中
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();   //feats_down_size；记录当前观测帧点云数
            
            /*** 初始化地图 构建kd树 ***/
            if(!init_map)
            {   
                // if(ikdtree.Root_Node == nullptr) //
                // // if(feats_down_size > 5)
                // {   // 设置ikd tree的降采样参数
                //     ikdtree.set_downsample_param(filter_size_map_min);
                // }
                
                // feats_down_world->resize(feats_down_size);//初始为加载地图时，以观测到的第一帧为准
                // for(int i = 0; i < feats_down_size; i++)
                // {   
                //     // 转到世界坐标系，输入feats_down_body->points[i]，输出feats_down_world->points[i]
                //     pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                // }
                // for (size_t i = 0; i < feats_down_world->size(); i++) 
                // {
                //     init_feats_world->points.emplace_back(feats_down_world->points[i]);
                // }
                // if(init_feats_world->size() < init_map_size) continue;

                //////////////////////////
                if( pcd_map_matching_en )
                {
                    // for (size_t i = 0; i < feats_down_world->size(); i++) 
                    // {
                    //     init_feats_world->points.emplace_back(feats_down_world->points[i]);
                    // }
                    
                    ;   // 由第一帧有效点云构建ikd-tree的操作在主函数开始就执行了
                }else
                {   
                    if(pcd_save_en)
                        RCLCPP_INFO(ros2_node->get_logger(),"运行SLAM模式,默认保存建图pcd于功能包路径下 slam_pcd_maps/slam_scans.pcd");
                    else 
                        RCLCPP_INFO(ros2_node->get_logger(),"运行SLAM模式,默认不保存建图pcd文件");

                    feats_down_world->resize(feats_down_size);//初始为加载地图时，以观测到的第一帧为准
                    for(int i = 0; i < feats_down_size; i++)
                    {   
                        // 转到世界坐标系，输入 feats_down_body->points[i]，输出feats_down_world->points[i]
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    for (size_t i = 0; i < feats_down_world->size(); i++) 
                    {
                        init_feats_world->points.emplace_back(feats_down_world->points[i]);
                    }
                    if(init_feats_world->size() < init_map_size) continue;
                    TicToc t_build;
                    ikdtree.Build(init_feats_world->points); 
                    std::cout<<"waste "<<t_build.toc()/1000.000<<" s\n";

                }
                
                //////////////////////////

                // // 组织ikd tree
                // ikdtree.Build(init_feats_world->points); 
                // std::cout<<"ikdtree Build\n";
                init_map = true;
                
                // publish_init_kdtree(pubLaserCloudMap); //(pubLaserCloudFullRes);
                continue;
            }        
            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);  // crossmat_list空间分配
            pbody_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);
                          
            // 将降采样后的点云数据转换成向量，并生成了相应的叉积矩阵
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {   // 遍历每个下采样后的点，将点的坐标存储到向量 pbody_list[i] 中
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                if (extrinsic_est_en) // 如果启用了外参估计（ extrinsic_est_en 为真 ），
                {   // 根据输入的 IMU 数据以及当前使用的卡尔曼滤波器状态，将点坐标从IMU坐标系变换到世界坐标系
                    if (!use_imu_as_input)
                    {
                        point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                    }
                    else
                    {
                        point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                    }
                }
                else//如果未启用外参估计，则将点坐标从Lidar坐标系变换到IMU坐标系，
                {   //这里 Lidar_R_wrt_IMU 表示Lidar坐标系相对于IMU坐标系的旋转矩阵，
                    // Lidar_T_wrt_IMU 表示Lidar坐标系相对于IMU坐标系的平移向量。
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                //根据向量 pbody_list[i] 的值计算叉积矩阵 point_crossmat，
                //并将其存储到容器 crossmat_list 中，最终生成了与下采样后的点云数据相对应的向量和叉积矩阵的列表。
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i]=point_crossmat;
            }
            
            average_distance_score = 0.0;
            effective_score_points_num = 0;
            overlap_points_num =0;

            if (!use_imu_as_input)
            {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** 逐点更新 point by point update ****/

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {   // 逐点
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    // 根据当前点的时间戳，计算当前时刻距离上一次预测或更新的时间间隔，更新时间戳
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame)
                    {
                        if(imu_en)
                        {
                            while (time_current > time_to_double(imu_next.header.stamp))
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                                // imu_deque.pop();
                            }

                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }

                    if(imu_en)
                    {
                        bool imu_comes = time_current > time_to_double(imu_next.header.stamp);
                        while (imu_comes) 
                        {
                            imu_upda_cov = true;
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            double dt = time_to_double(imu_last.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = time_to_double(imu_last.header.stamp); // big problem
                            imu_comes = time_current > time_to_double(imu_next.header.stamp);
                            // if (!imu_comes)
                            //判断当前点的时间戳是否大于IMU数据的时间戳，并在IMU数据可用的情况下对卡尔曼滤波器的状态进行更新
                            //即调用卡尔曼滤波器的更新函数 update_iterated_dyn_share_IMU()
                            {
                                double dt_cov = time_to_double(imu_last.header.stamp) - time_update_last; 

                                if (dt_cov > 0.0)
                                {
                                    // time_update_last = imu_last.header.stamp.toSec();
                                    time_update_last = time_to_double(imu_last.header.stamp);

                                    double propag_imu_start = omp_get_wtime();

                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }// 特别地，如果没有IMU数据，则只进行状态预测，并在需要时对卡尔曼滤波器的状态进行更新

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1)
                    {
                        RCLCPP_WARN(ros2_node->get_logger(),"No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }else   // 评分机制
                    {
                        if(pointSearchSqDis.size() > 0 && pointSearchSqDis.empty() == false)
                        {
                            effective_score_points_num ++;
                            if(average_distance_score <= 3600.00)   // 这里还不是平均距离，是所有近邻点的距离之和，要是超过 3600 肯定寄了
                                average_distance_score += pointSearchSqDis.front();
                            if(pointSearchSqDis.front() < 0.005)  // 如果跟地图点对比小于 5mm,认为重叠
                            {
                                overlap_points_num ++;
                            }
                        }

                    }
                    
                    if(prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (!imu_en && (dt_cov >= imu_time_inte)) // (point_cov_not_prop && imu_prop_cov)
                        {
                            double propag_cov_start = omp_get_wtime();
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            imu_upda_cov = false;
                            time_update_last = time_current;
                            propag_time += omp_get_wtime() - propag_cov_start;
                        }
                    }


                    solve_start = omp_get_wtime();
                    if(relocation_set_from_outside_flag == 1) 
                    {
                        set_init_poses(relocation_pose_from_outside);

                        nav_msgs::msg::Odometry aaa;
                        aaa.pose.pose.position.x=5.0;
                        aaa.pose.pose.position.y=3.0;
                        aaa.pose.pose.position.z=0;
                        aaa.pose.pose.orientation.x=0;
                        aaa.pose.pose.orientation.y=0;
                        aaa.pose.pose.orientation.z=0;
                        aaa.pose.pose.orientation.w=1;
                        set_init_poses(aaa);std::cout<<"使用前值重定位...\n";
                        relocation_set_from_outside_flag = 2;
                    } 
                    
                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/

                        // publish_odometry(pubOdomAftMapped,ros2_node,pubOdomAftMapped_2,pubLinearVelAndAcc);
                        publish_LAM(LAM_publisher,pubOdomForDebug,ros2_node);
                        if (runtime_pos_log)
                        {   
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                            <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }
                    //根据卡尔曼滤波器更新后的状态，将下采样后的点云数据转换到世界坐标系中，即 pointBodyToWorld()
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                
                    solve_time += omp_get_wtime() - solve_start;
    
                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
            }
            else
            {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame)
                    {
                        while (time_current > time_to_double(imu_next.header.stamp)) 
                        {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }
                        imu_prop_cov = true;
                        // imu_upda_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current; 
                        // if(prop_at_freq_of_imu)
                        {
                            input_in.gyro<<imu_last.angular_velocity.x,
                                        imu_last.angular_velocity.y,
                                        imu_last.angular_velocity.z;
                                            
                            input_in.acc<<imu_last.linear_acceleration.x,
                                        imu_last.linear_acceleration.y,
                                        imu_last.linear_acceleration.z;
                            // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                            //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                            //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);
                                            
                            // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                            //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                                        // 0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);

                            // angvel_avr -= state.bias_g;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }
                    
                    while (time_current > time_to_double(imu_next.header.stamp)) // && !imu_deque.empty())
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z; 

                        // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                        //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                        //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);
                                        
                        // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                        //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                        //             0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);
                        input_in.acc    = input_in.acc * G_m_s2 / acc_norm; 
                        double dt = time_to_double(imu_last.header.stamp) - t_last;

                        // if(!prop_at_freq_of_imu)
                        // {       
                        double dt_cov = time_to_double(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = time_to_double(imu_last.header.stamp); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false); 
                        t_last = time_to_double(imu_last.header.stamp);
                        imu_prop_cov = true;
                        // imu_upda_cov = true;
                    }      

                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();
                    
                    if(!prop_at_freq_of_imu)
                    {   
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {    
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = time_current; 
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false); 

                    propag_time += omp_get_wtime() - propag_start;

                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();
                    
                    if (feats_down_size < 1)
                    {
                        // ROS_WARN("No point, skip this scan!\n");
                        RCLCPP_WARN(ros2_node->get_logger(),"No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    // if(prop_at_freq_of_imu)
                    // {
                    //     double dt_cov = time_current - time_update_last;
                    //     if ((imu_prop_cov && dt_cov > 0.0) || (dt_cov >= imu_time_inte * 1.2)) 
                    //     {
                    //         double propag_cov_start = omp_get_wtime();
                    //         kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                    //         propag_time += omp_get_wtime() - propag_cov_start;
                    //         time_update_last = time_current;
                    //         imu_prop_cov = false;
                    //     }
                    // }
                    if (publish_odometry_without_downsample)    // 高帧率发布
                    {
                        /******* Publish odometry *******/

                        // publish_odometry(pubOdomAftMapped,ros2_node,pubOdomAftMapped_2,pubLinearVelAndAcc);
                        publish_LAM(LAM_publisher,pubOdomForDebug,ros2_node);
                        if (runtime_pos_log)
                        {   
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                            <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j); 
                    }
                    solve_time += omp_get_wtime() - solve_start;
                
                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }  
            }

            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample)   // 低帧率时的发布
            {
                // publish_odometry(pubOdomAftMapped,ros2_node,pubOdomAftMapped_2,pubLinearVelAndAcc);
                publish_LAM(LAM_publisher,pubOdomForDebug,ros2_node);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            
            if(feats_down_size > 4)
            {   
                if( !(pcd_map_matching_en) )
                {   // mapping模式，不进行匹配，全用于建图
                    map_incremental();
                    average_distance_score=0.0001;

                }else
                {   
                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    // pcl::PointCloud<PointType>::Ptr measure_cloud(new pcl::PointCloud<PointType>());
                    // measure_cloud = feats_down_world;

                    // // 定位位姿转换eigen 4f ////////////
                    // // Convert quaternion to rotation matrix
                    // Eigen::Quaternionf q (
                    //                         kf_output.x_.rot.coeffs()[0],
                    //                         kf_output.x_.rot.coeffs()[1], 
                    //                         kf_output.x_.rot.coeffs()[2], 
                    //                         kf_output.x_.rot.coeffs()[3]
                    //                      );
                    // Eigen::Matrix3f rotation_matrix = q.normalized().toRotationMatrix();

                    // // Fill the transform matrix
                    // transform.block<3, 3>(0, 0) = rotation_matrix;
                    // transform(0, 3) = kf_output.x_.pos(0);
                    // transform(1, 3) = kf_output.x_.pos(1);
                    // transform(2, 3) = kf_output.x_.pos(2);
                    // // 定位位姿转换eigen 4f ////////////
                    
                        // PointVector PointToAdd;  // 需要在ikd-tree地图中新增的点云
                        // PointVector PointNoNeedDownsample;  //加入ikd-tree时，不需要降采样的点云
                        // PointToAdd.reserve(feats_down_size);
                        // PointNoNeedDownsample.reserve(feats_down_size);
                        // // std::shared_ptr<pcl::PointXYZI> feats_down_world_same = std::make_shared<pcl::PointXYZI>();
                        // PointCloudXYZI::Ptr feats_down_world_same(new PointCloudXYZI());

                        // for(int i = 0; i < feats_down_size; i++)
                        // {  
                        //     /* decide if need add to map */
                        //     if (!Nearest_Points[i].empty())
                        //     {
                        //         const PointVector &points_near = Nearest_Points[i];
                        //         bool need_add = true;

                        //         PointType downsample_result, mid_point; 
                        //         // 体素滤波器长度：filter_size_map_min，这个参数在fast-lio中设为0.1,在point-lio就比较大
                        //         // mid_point即为该特征点所属的栅格的中心点坐标
                        //         mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                        //         mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                        //         mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                        //         /* If the nearest points is definitely outside the downsample box */
                        //         if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min)
                        //         {
                        //             // 若三个方向距离都大于地图栅格半轴长，无需降采样
                        //             // 如果最近的点 points_near[0]在体素格子之外，那么就直接将该特征点添加到PointNoNeedDownsample向量，并跳过当前循环
                        //             PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                        //             continue;
                        //         }
                        //         /* Check if there is a point already in the downsample box and closer to the center point */
                        //         // 二范数：dist
                        //         float dist  = calc_dist<float>(feats_down_world->points[i],mid_point);
                        //         for (int readd_i = 0; readd_i < points_near.size(); readd_i ++)
                        //         {
                        //             // if (points_near.size() < NUM_MATCH_POINTS) break;
                        //             /* Those points which are outside the downsample box should not be considered. */
                        //             // if (fabs(points_near[readd_i].x - mid_point.x) > 0.5 * filter_size_map_min || fabs(points_near[readd_i].y - mid_point.y) > 0.5 * filter_size_map_min || fabs(points_near[readd_i].z - mid_point.z) > 0.5 * filter_size_map_min) {
                        //             //     continue;                    
                        //             // }
                        //             // if (calc_dist<float>(points_near[readd_i], mid_point) < dist)
                        //             if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) 
                        //             {
                        //                 need_add = false;
                        //                 feats_down_world_same->points.emplace_back(feats_down_world->points[i]);
                        //                 break;
                        //             }
                        //         }
                        //         if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
                        //     }
                        //     else  // 初始点
                        //     {
                        //         // PointToAdd.emplace_back(feats_down_world->points[i]);
                        //         PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                        //     }
                        // }
                    

                    // ComputeScore(ikdtree,feats_down_world,average_distance_score,overlap_score);
                    overlap_score = overlap_points_num / (double)effective_score_points_num;
                    average_distance_score = average_distance_score / (double)effective_score_points_num;

                    // std::cout<<"feats_down_world size: "<<feats_down_world->points.size()<<endl;
                    // std::cout<<"NearPoints size: "<<Nearest_Points.size()<<endl;
                    // std::cout<<"有效距离点数: "<<effective_score_points_num<<endl;
                    // std::cout<<"重叠点数: "<<overlap_points_num<<endl;
                    

                    // cout<<"DistanceScore: "<< average_distance_score 
                    // <<"\t OverlapScore: "<<overlap_score << endl;
                    // cout<<"feats_down_world_same size: "<<feats_down_world_same->points.size()<< endl;
                    
                    // overlap_score_thr = 0.97;
                    static double last_overlap_score = overlap_score;
                    if(average_distance_score < 0.0025 && overlap_score>overlap_score_thr&&last_overlap_score>overlap_score_thr)
                        map_incremental();
                    last_overlap_score = overlap_score;

                        // ikdtree.Add_Points(PointToAdd, true),
                    //     ikdtree.Add_Points(PointNoNeedDownsample, false);// ikd-tree维护的点云地图加入
                    // else if(average_distance_score < 0.005&& overlap_score>0.99)
                    //     // map_incremental();
                    //     ikdtree.Add_Points(PointToAdd, true),
                    //     ikdtree.Add_Points(PointNoNeedDownsample, false);// ikd-tree维护的点云地图加入
                    // else if(average_distance_score < 0.003&& overlap_score>0.98)
                    //     // map_incremental();
                    //     ikdtree.Add_Points(PointToAdd, true),
                    //     ikdtree.Add_Points(PointNoNeedDownsample, false);// ikd-tree维护的点云地图加入
                    const short int index_length = 10;
                    static double dis_scores[index_length] = {0.00};  // 用于存储最近index_length次的距离分数,初始化所有元素为0
                    static double olap_scores[index_length] = {0.00};
                    static int score_index = 0;
                    if(score_index%index_length == 0)
                        score_index = 0;
                    else 
                        score_index ++;
                    dis_scores[score_index] = average_distance_score;
                    olap_scores[score_index] = overlap_score;

                    // 计算 dis_scores 的均值
                    aver_dis_scores = 0.0;
                    for (short int i = 0; i < index_length; i++) 
                    {
                        aver_dis_scores += dis_scores[i];
                    }
                    aver_dis_scores = aver_dis_scores/(double)index_length;
                    // 计算 olap_scores 的均值
                    aver_olap_scores = 0.0;
                    for (size_t i = 0; i < index_length; i++) 
                    {
                        aver_olap_scores += olap_scores[i];
                    }
                    double olap_scores_avg = aver_olap_scores / (double)index_length;

                    // std::cout<<"aver_dis_scores: "<<aver_dis_scores<<endl;
                    // std::cout<<"aver_olap_scores: "<<aver_olap_scores<<endl;
                    static int relocalization_counter_times = 0;
                    if(aver_dis_scores>0.1||aver_olap_scores<0.60)
                    {       
                            yellow_info("LAM判定定位丢失, 重定位中...");
                            
                            if(relocalization_counter_times>20)
                            relocalization_set(last_true_LAM_pose);
                            relocalization_counter_times++;
                            LAM_trustable = false;
                        
                    }else 
                        {   relocalization_counter_times=0;
                            LAM_trustable = true;
                            
                        }
                    
                }
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (pcd_map_matching_en&&(scan_pub_en || pcd_save_en)&&overlap_score > overlap_score_thr &&average_distance_score < 0.0025)
                publish_frame_world(pubLaserCloudFullRes);
            if(!pcd_map_matching_en&&(scan_pub_en || pcd_save_en)) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            
            /*** Debug variables Logging ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num; // 每帧点云在框架中计算所用平均时间
                {aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;} // 每帧点云ICP迭代所用平均时间
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu, aver_time_icp, aver_time_propag); 
                if (!publish_odometry_without_downsample)
                {
                    if (!use_imu_as_input)
                    {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                        <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                    else
                    {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                        <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        status = rclcpp::ok();
        rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if(pcd_map_matching_en)
        store_world_for_matching();
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {   
        string file_name = string("slam_scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "slam_pcd_maps/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
