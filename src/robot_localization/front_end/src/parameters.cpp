#include "parameters.h"
#include "localization_tools/color_terminal.hpp"
bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;

std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
int  init_map_size, con_frame_num;
double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
float  plane_thr;
double filter_size_surf_min, filter_size_map_min, fov_deg;
double cube_len; 
float  DET_RANGE;
bool   imu_en;
bool gravity_align;
double imu_time_inte;
double laser_point_cov, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init;
std::vector<double> gravity;

std::vector<double> extrinT;
std::vector<double> extrinR;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;
bool pcd_map_matching_en; 
std::string yaml_config_path;
std::string matching_pcd_map_path;
double overlap_score_thr;
Eigen::Matrix3d ImuMount_compensate_matrix3d=Eigen::Matrix3d::Identity();

void readParameters(std::shared_ptr<rclcpp::Node>& node_)
{
  // cout<<"to read parameters\n";
  p_pre.reset(new Preprocess());
  prop_at_freq_of_imu = node_->declare_parameter<bool>("prop_at_freq_of_imu",1);
  use_imu_as_input = node_->declare_parameter<bool>("use_imu_as_input",1);
  check_satu = node_->declare_parameter<bool>("check_satu",1);
  init_map_size = node_->declare_parameter<int>("init_map_size",100);
  space_down_sample = node_->declare_parameter<bool>("space_down_sample",1);
  satu_acc = node_->declare_parameter<double>("mapping.satu_acc",3.0);
  satu_gyro = node_->declare_parameter<double>("mapping.satu_gyro",35.0);
  acc_norm = node_->declare_parameter<double>("mapping.acc_norm", 1.0);
  plane_thr = node_->declare_parameter<float>("mapping.plane_thr", 0.05f);
  p_pre->point_filter_num = node_->declare_parameter<int>("point_filter_num", 2);
  lid_topic = node_->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  imu_topic = node_->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  con_frame = node_->declare_parameter<bool>("common.con_frame", false);
  con_frame_num = node_->declare_parameter<int>("common.con_frame_num", 1);
  cut_frame = node_->declare_parameter<bool>("common.cut_frame", false);
  cut_frame_time_interval = node_->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
  time_lag_imu_to_lidar = node_->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.0);
  filter_size_surf_min = node_->declare_parameter<double>("filter_size_surf", 0.5);
  filter_size_map_min = node_->declare_parameter<double>("filter_size_map", 0.5);
  cube_len = node_->declare_parameter<double>("cube_side_length", 200.0);
  DET_RANGE = node_->declare_parameter<float>("mapping.det_range", 300.f);
  fov_deg = node_->declare_parameter<double>("mapping.fov_degree", 180.0);
  imu_en = node_->declare_parameter<bool>("mapping.imu_en", true);
  extrinsic_est_en = node_->declare_parameter<bool>("mapping.extrinsic_est_en", true);
  imu_time_inte = node_->declare_parameter<double>("mapping.imu_time_inte", 0.005);
  laser_point_cov = node_->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
  acc_cov_input = node_->declare_parameter<double>("mapping.acc_cov_input", 0.1);
  vel_cov = node_->declare_parameter<double>("mapping.vel_cov", 20.0);
  gyr_cov_input = node_->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
  gyr_cov_output = node_->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
  acc_cov_output = node_->declare_parameter<double>("mapping.acc_cov_output", 0.1);
  b_gyr_cov = node_->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
  b_acc_cov = node_->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
  imu_meas_acc_cov = node_->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov = node_->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
  p_pre->blind = node_->declare_parameter<double>("preprocess.blind", 1.0);
  lidar_type = node_->declare_parameter<int>("preprocess.lidar_type", 1);
  p_pre->N_SCANS = node_->declare_parameter<int>("preprocess.scan_line", 16);
  p_pre->SCAN_RATE = node_->declare_parameter<int>("preprocess.scan_rate", 10);
  p_pre->time_unit = node_->declare_parameter<int>("preprocess.timestamp_unit", 1);
  match_s = node_->declare_parameter<double>("mapping.match_s", 81.0);
  // gravity_init = node_->declare_parameter<std::vector<double>>("mapping.gravity", {0.0, 0.0, -9.810});

  gravity_align = node_->declare_parameter<bool>("mapping.gravity_align", true);
  gravity = node_->declare_parameter<std::vector<double>>("mapping.gravity", {0.0, 0.0, -9.810});
  gravity_init = node_->declare_parameter<std::vector<double>>("mapping.gravity_init", {0.0, 0.0, -9.810});
  

  extrinT = node_->declare_parameter<std::vector<double>>("mapping.extrinsic_T", { -0.011, -0.0234, 0.044});
  extrinR = node_->declare_parameter<std::vector<double>>("mapping.extrinsic_R", { 1, 0, 0,
                                                                                   0, 1, 0,
                                                                                   0, 0, 1 });
  publish_odometry_without_downsample = node_->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
  path_en = node_->declare_parameter<bool>("publish.path_en", true);
  scan_pub_en = node_->declare_parameter<bool>("publish.scan_publish_en", true);
  scan_body_pub_en = node_->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
  runtime_pos_log = node_->declare_parameter<bool>("runtime_pos_log_enable", false);
  pcd_save_en = node_->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  pcd_save_interval = node_->declare_parameter<int>("pcd_save.interval", -1);

  pcd_map_matching_en = node_->declare_parameter<bool>("pcd_map_matching_en", false);
  yaml_config_path = node_->declare_parameter<std::string>("yaml_config_path", "");
  matching_pcd_map_path = node_->declare_parameter<std::string>("matching_pcd_map_path", "");
  overlap_score_thr = node_->declare_parameter<double>("overlap_score_thr", 0.9876);
  std::vector<double> ImuMount_compensate_degrees = node_->declare_parameter<std::vector<double>>("ImuMount_compensate_degrees", {0.0, 0.0, 0.0});
  if(ImuMount_compensate_degrees.size() == 3)
  { 
    double roll = ImuMount_compensate_degrees[0];
    double pitch = ImuMount_compensate_degrees[1];
    double yaw = ImuMount_compensate_degrees[2];

    green_info("载入Mount_compensate, roll:"+std::to_string(roll)
              +", pitch:"+std::to_string(pitch)
              +", yaw:"+std::to_string(yaw));

    // 分别计算x轴和y轴、z轴的旋转矩阵
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    Eigen::Matrix3d Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;
    // 计算安装旋转矩阵
    ImuMount_compensate_matrix3d = Rx * Ry * Rz;
  }


}