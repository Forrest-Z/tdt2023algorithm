/*
 * @Description: 用于该包全局路径定义，以便于获取制定的文件
 */
#ifndef ROBOT_LOCALIZATION_GLOBAL_DEFINATION_H_IN_
#define ROBOT_LOCALIZATION_GLOBAL_DEFINATION_H_IN_

#include <string>

#define PROJECT_PATH "/home/tdt-link/WorkSpace/Github/tdt2023LAM/src/robot_localization/robot_localization"
// 这里看到的绝对路径不需要人为去改，编译即会生成对应于设备的路径
namespace robot_localization
{   
    const std::string LOCALIZATION_PACKAGE_PATH = "/home/tdt-link/WorkSpace/Github/tdt2023LAM/src/robot_localization/robot_localization";
    const std::string WORK_PACKAGE_PATH = "/home/tdt-link/WorkSpace/Github/tdt2023LAM/src/robot_localization/robot_localization";
    const std::string WORK_SPACE_PATH = "/home/tdt-link/WorkSpace/Github/tdt2023LAM/src/robot_localization/robot_localization";
}

#endif
