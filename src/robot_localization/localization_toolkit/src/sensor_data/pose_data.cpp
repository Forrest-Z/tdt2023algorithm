/*
 * @Description: 
 */
// #include "../../include/sensor_data/pose_data.hpp"
#include "sensor_data/pose_data.hpp"
namespace robot_localization {

Eigen::Quaternionf PoseData::GetQuaternion() 
{
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}


}