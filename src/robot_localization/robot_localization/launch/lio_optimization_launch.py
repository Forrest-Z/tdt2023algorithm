import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

keyframe_meter_gap=0.001
mapviz_filter_size=0.032
sc_dist_thres=0.00005
sc_max_radius=40.00

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization/robot_localization')

                                
    save_directory= pkg_path + '/optimization_data/'


    sc_pgo_optimizing_end_node = Node(         # Scan Context-based PGO
                                    package="robot_localization",
                                    executable='alaserPGO',
                                    name='alaserPGO',
                                    output='screen',emulate_tty=True,
                                    parameters = [ {'keyframe_meter_gap': keyframe_meter_gap},
                                                   {'mapviz_filter_size': mapviz_filter_size},
                                                   {'sc_dist_thres': sc_dist_thres},
                                                   {'sc_max_radius': sc_max_radius},
                                                   {'save_directory': save_directory}
                                                  
                                                  
                                                 ],
                                    
                                )     
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/config/rviz/lio_optimization_config.rviz']

                     )                     
    
    
    ld.add_action(sc_pgo_optimizing_end_node)
    # ld.add_action(data_pretreat_end_node)
    # ld.add_action(lidar_odom_end_node)
    # ld.add_action(start_rviz2_node)


    return ld
