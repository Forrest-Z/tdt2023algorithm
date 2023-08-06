from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

use_imu_as_input = False
prop_at_freq_of_imu = True
check_satu = True
init_map_size = 10
point_filter_num = 4
space_down_sample = True
filter_size_surf = 0.4          # 原为 0.5
filter_size_map = 0.4           # 原为 0.5
cube_side_length = 1000.00
runtime_pos_log_enable = False

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='front_end').find('front_end') 
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization/front_end')
    # 构建YAML文件的完整路径
    YamlPath = os.path.join(FindPackageShare(package='front_end').find('front_end'), 'config', 'velodyne.yaml')
    
    load_velodyne_config_cmd = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/laserMapping',YamlPath],
        output='screen',emulate_tty=True)

    front_end_slam_node = Node(
        package = 'front_end',
        executable = 'frontend_mapping',
        name = 'laserMapping',
        output = 'screen',emulate_tty=True,
        parameters = [
            {'use_imu_as_input': use_imu_as_input},
            {'prop_at_freq_of_imu': prop_at_freq_of_imu},
            {'check_satu': check_satu},
            {'init_map_size': init_map_size},
            {'point_filter_num': point_filter_num},
            {'space_down_sample': space_down_sample},
            {'filter_size_surf': filter_size_surf},
            {'filter_size_map': filter_size_map},
            {'cube_side_length': cube_side_length},
            {'runtime_pos_log_enable': runtime_pos_log_enable},
            
            YamlPath
            ] 
        )
    
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/rviz_cfg/lio.rviz']

                     )
    

    return LaunchDescription(
        [
        #   load_velodyne_config_cmd,
          front_end_slam_node,
          start_rviz2_node,

        ]
    )


