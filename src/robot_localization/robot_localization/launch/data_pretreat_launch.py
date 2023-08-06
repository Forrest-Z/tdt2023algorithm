import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    yaml_config_path = pkg_share + '/config/livox_lidar.yaml'      

    
    
    data_pretreat_end_node = Node(
                                package="robot_localization",
                                executable='data_pretreat_end_node',
                                name='data_pretreat_end_node',
                                output='screen',emulate_tty=True,
                                parameters = [ {'yaml_config_path': yaml_config_path}, ] 

    )   


    ld.add_action(data_pretreat_end_node)


    return ld
