import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

pcd_process_action = 1    # pcd_process_action=2则分割 src/robot_localization/pcd_maps/input_matching_cloud.pcd 为10个子pcd，
                          # =1时为坐标转换
                          

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization')
    

    pcd_process_end_node = Node(
                                package="robot_localization",
                                executable='pcd_process_end',
                                output='screen',emulate_tty=True,
                                parameters = [ {'pcd_process_action': pcd_process_action} ] 

                              )
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/config/rviz/3d_lidar.rviz']

                     )
    
    
    ld.add_action(pcd_process_end_node)

    ld.add_action(start_rviz2_node)


    return ld


