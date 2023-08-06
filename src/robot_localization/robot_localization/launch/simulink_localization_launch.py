import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization/robot_localization')
    
    use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

    start_simulation_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('genshin_simulation'), 'launch', 'HDL32_car_launch.py')])
             )
    data_pretreat_end_node = Node(
                                package="robot_localization",
                                executable='data_pretreat_end_node',
                                output='screen',emulate_tty=True,
                                parameters=[{'use_sim_time': True}]
    )   

    lidar_odom_end_node = Node(
                                package="robot_localization",
                                executable='lidar_odom_end_node',
                                output='screen',emulate_tty=True,
                                parameters=[{'use_sim_time': True}]
                              )
    pcd_matching_end_node = Node(
                                    package="robot_localization",
                                    executable='matching_end_node',
                                    output='screen',emulate_tty=True,
                                    parameters=[{'use_sim_time': True}]
                                )
    graph_optimizing_end_node = Node(
                                    package="robot_localization",
                                    executable='optimization_end_node',
                                    parameters=[{'use_sim_time': True}]
                                )             
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/config/rviz/sim_3d_lidar.rviz'],
                        parameters=[{'use_sim_time': True}]

                     )

    
    pcd_showin_rviz_node = Node(
                                package="robot_localization",
                                executable='pcd_process_end',
                                output='screen',emulate_tty=True,
                                parameters=[{'use_sim_time': True}]
                              )
    


    # ld.add_action(use_sim_time)
    ld.add_action(start_simulation_cmd)
    ld.add_action(start_rviz2_node)
    ld.add_action(data_pretreat_end_node)
    ld.add_action(lidar_odom_end_node)
    # ld.add_action(pcd_matching_end_node)
    
    # ld.add_action(graph_optimizing_end_node)
    ld.add_action(pcd_showin_rviz_node)

    return ld