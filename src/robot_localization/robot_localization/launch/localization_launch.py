import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization/robot_localization')
    yaml_config_path = pkg_share + '/config/livox_lidar.yaml'      

    start_mid360_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
             )
                                
    
    
    
    data_pretreat_end_node = Node(
                                package="robot_localization",
                                executable='data_pretreat_end_node',
                                output='screen',emulate_tty=True,
                                parameters = [ {'yaml_config_path': yaml_config_path} ] 

    )   

    lidar_odom_end_node = Node(
                                package="robot_localization",
                                executable='lidar_odom_end_node',
                                output='screen',emulate_tty=True
                              )
    pcd_matching_end_node = Node(
                                    package="robot_localization",
                                    executable='matching_end_node',
                                    output='screen',emulate_tty=True
                                )
    graph_optimizing_end_node = Node(
                                    package="robot_localization",
                                    executable='optimization_end_node',
                                    output='screen',emulate_tty=True
                                )                          
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/config/rviz/3d_lidar.rviz']

                     )
        
    pcd_process_node = Node(
                                package="robot_localization",
                                executable='pcd_process_end',
                                output='screen',emulate_tty=True
                              )
    
    start_frontend_matching_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('front_end'), 'launch', 'matching_mid360_launch.py')]),
                
             )
    start_frontend_slam = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('front_end'), 'launch', 'mapping_mid360_launch.py')])
                    )
    start_lio_optimization_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot_localization'), 'launch', 'lio_optimization_launch.py')]),
                
             )
    
    ld.add_action(start_mid360_driver)
    # ld.add_action(data_pretreat_end_node)
    ld.add_action(start_frontend_slam)

    # ld.add_action(pcd_matching_end_node)
    # ld.add_action(start_lio_optimization_cmd)
    # ld.add_action(pcd_process_node )

    # ld.add_action(start_frontend_matching_cmd)
    # ld.add_action(start_rviz2_node)


    return ld
