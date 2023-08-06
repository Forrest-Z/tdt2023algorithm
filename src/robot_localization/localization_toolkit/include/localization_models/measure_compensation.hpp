/*
 * @Description: 观测值补偿
 */

#ifndef LOCALIZATION_MODELS_MEASURE_COMPENSATION_HPP_
#define LOCALIZATION_MODELS_MEASURE_COMPENSATION_HPP_

#include <pcl/filters/crop_box.h>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

typedef Eigen::Vector3d V3D;

// // 计算roll和pitch角
// inline void compute_orientation(const sensor_msgs::msg::Imu measure_msg, double roll, double pitch)
// {   
    
//     // roll = atan2(ay, sqrt(ax * ax + az * az));
//     // pitch = atan2(-ax, sqrt(ay * ay + az * az));
//     roll = atan2(measure_msg.linear_acceleration.y, sqrt(measure_msg.linear_acceleration.x * measure_msg.linear_acceleration.x + measure_msg.linear_acceleration.z * measure_msg.linear_acceleration.z));
//     pitch = atan2(-measure_msg.linear_acceleration.x, sqrt(measure_msg.linear_acceleration.y * measure_msg.linear_acceleration.y + measure_msg.linear_acceleration.z * measure_msg.linear_acceleration.z));
// }

// 输入测量值，计算roll和pitch角，计算安装旋转矩阵
inline Eigen::Matrix3d getImuMountRotationMatrix(const sensor_msgs::msg::Imu measure_msg) 
{   
    double roll,pitch;
    roll = atan2(measure_msg.linear_acceleration.y, sqrt(measure_msg.linear_acceleration.x * measure_msg.linear_acceleration.x + measure_msg.linear_acceleration.z * measure_msg.linear_acceleration.z));
    pitch = atan2(-measure_msg.linear_acceleration.x, sqrt(measure_msg.linear_acceleration.y * measure_msg.linear_acceleration.y + measure_msg.linear_acceleration.z * measure_msg.linear_acceleration.z));


    // 分别计算x轴和y轴的旋转矩阵
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    Eigen::Matrix3d Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    // 计算安装旋转矩阵
    Eigen::Matrix3d R = Rx * Ry;
    return R;
}

inline Eigen::Matrix3d getImuMountRotationMatrix(const Eigen::Vector3d measure_msg) 
{   
    double roll,pitch;
    roll = atan2(measure_msg.y(), sqrt(measure_msg.x() * measure_msg.x() + measure_msg.z() * measure_msg.z()));
    pitch = atan2(-measure_msg.x(), sqrt(measure_msg.y() * measure_msg.y() + measure_msg.z() * measure_msg.z()));
      
      std::cout << "在pitch方向补偿弧度为: " <<  pitch << " rad" << std::endl;
    
    // 转换为角度
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    std::cout << "在roll 方向(绕x轴)应补偿弧度为: "<< roll << " rad,角度为："<< roll_deg << " degrees"  << std::endl;
    std::cout << "在pitch方向(绕y轴)应补偿弧度为: " <<  pitch << " rad,角度为："<< pitch_deg << " degrees" << std::endl;

    // 分别计算x轴和y轴的旋转矩阵
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    Eigen::Matrix3d Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    // 计算安装旋转矩阵
    Eigen::Matrix3d R = Rx * Ry;
    return R;
}
inline void compensateImuMeasurement(
                                     double &linear_acc_x,
                                     double &linear_acc_y,
                                     double &linear_acc_z,
                                     const Eigen::Matrix3d& imu_mount_rotation                                    
                                    ) 
{
    // 将IMU消息的加速度和角速度数据转换为Eigen::Vector3d
    Eigen::Vector3d accel(linear_acc_x, linear_acc_y, linear_acc_z);
    // 角速度的矫正暂时不需要
    // Eigen::Vector3d gyro(measurement.angular_velocity.x, measurement.angular_velocity.y, measurement.angular_velocity.z);

    // 使用安装旋转矩阵补偿加速度和角速度
    Eigen::Vector3d compensated_accel = imu_mount_rotation * accel;
    // Eigen::Vector3d compensated_gyro = imu_mount_rotation * gyro;

    // 将补偿后的加速度和角速度数据重新转换为sensor_msgs::msg::Imu

    linear_acc_x = compensated_accel(0);
    linear_acc_y = compensated_accel(1);
    linear_acc_z = compensated_accel(2);

    // measurement.angular_velocity.x = compensated_gyro(0);
    // measurement.angular_velocity.y = compensated_gyro(1);
    // measurement.angular_velocity.z = compensated_gyro(2);

}

inline void compensateImuMeasurement(
                                     sensor_msgs::msg::Imu &msg,
                                     const Eigen::Matrix3d& imu_mount_rotation                                    
                                    ) 
{
    // 将IMU消息的加速度和角速度数据转换为Eigen::Vector3d
    Eigen::Vector3d accel(msg.linear_acceleration.x , msg.linear_acceleration.y, msg.linear_acceleration.z);

    Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    // 使用安装旋转矩阵补偿加速度和角速度
    Eigen::Vector3d compensated_accel = imu_mount_rotation * accel;
    Eigen::Vector3d compensated_gyro = imu_mount_rotation * gyro;

      msg.linear_acceleration.x = compensated_accel(0);
      msg.linear_acceleration.y = compensated_accel(1);
      msg.linear_acceleration.z = compensated_accel(2);
      msg.angular_velocity.x = compensated_gyro(0);
      msg.angular_velocity.y = compensated_gyro(1);
      msg.angular_velocity.z = compensated_gyro(2);

}
inline void printImuMountRotationMatrix(const Eigen::Matrix3d& imu_mount_rotation)
{   
    // 从旋转矩阵中提取roll和pitch角
    double roll = atan2(imu_mount_rotation(2, 1), imu_mount_rotation(2, 2));
    double pitch = atan2(-imu_mount_rotation(2, 0), sqrt(imu_mount_rotation(2, 1) * imu_mount_rotation(2, 1) + imu_mount_rotation(2, 2) * imu_mount_rotation(2, 2)));
    // 转换为角度
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;

      std::cout << "在roll方向补偿角度为: "<< roll_deg << " degrees" << std::endl;
      std::cout << "在pitch方向补偿角度为: " <<  pitch_deg << " degrees" << std::endl;

}


/*
 @Description: 利用补偿矩阵，矫正由于雷达安装角度造成的测量值偏差，将所有观测点云矫正到重力水平面 
*/
typedef pcl::PointXYZINormal PointType;
inline void compensateLidarMeasurement(
                                          PointType const * const pi,
                                          PointType * const po,
                                          const Eigen::Matrix3d& imu_mount_rotation                                    
                                      ) 
{     
      V3D p_in(pi->x, pi->y, pi->z);
      
      V3D p_global;
      p_global = imu_mount_rotation * p_in;
      
      
      po->x = p_global(0);
      po->y = p_global(1);
      po->z = p_global(2);
      po->intensity = pi->intensity;
}

// 矫正点云
inline void correctPointCloud(pcl::PointCloud<pcl::PointXYZINormal>& cloud, const Eigen::Matrix3d& transformMatrix)
{
      double roll = atan2(transformMatrix(2, 1), transformMatrix(2, 2));
      double pitch = atan2(-transformMatrix(2, 0), sqrt(transformMatrix(2, 1) * transformMatrix(2, 1) + transformMatrix(2, 2) * transformMatrix(2, 2)));
      double roll_deg = roll * 180.0 / M_PI;
      double pitch_deg = pitch * 180.0 / M_PI;

      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      // transform.rotate(Eigen::AngleAxisf(5*roll, Eigen::Vector3f::UnitX()));

      transform.rotate(Eigen::AngleAxisf(pitch*(1), Eigen::Vector3f::UnitY()));
      transform.rotate(Eigen::AngleAxisf(roll*(1), Eigen::Vector3f::UnitX()));
      pcl::transformPointCloudWithNormals(cloud, cloud, transform);

//     // 遍历点云中的每个点
//     for (auto& point : cloud.points)
//     {
//         // 将点的坐标表示为Eigen向量
//         Eigen::Vector3d pointVector(point.x, point.y, point.z);
        
//         // 使用变换矩阵矫正点的坐标
//         Eigen::Vector3d correctedPoint = transformMatrix * pointVector;

//         // 更新矫正后的点的坐标
//         point.x = correctedPoint(0);
//         point.y = correctedPoint(1);
//         point.z = correctedPoint(2);
//     }
}
#endif 