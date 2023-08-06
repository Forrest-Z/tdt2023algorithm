import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

pcd_map_matching_en = True          # 是否开启点云图匹配模式,图匹配模式下同样能建图生成pcd；该模型切换应由launch启动文件执行

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='front_end').find('front_end') 
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization/front_end')
    # 构建YAML文件的完整路径
    YamlPath = os.path.join(FindPackageShare(package='front_end').find('front_end'), 'config', 'mid360.yaml')

    front_end_slam_node = Node(
        package = 'front_end',
        executable = 'frontend_mapping',
        name = 'laserMapping',
        output = 'screen',emulate_tty=True,
        parameters = [ {'pcd_map_matching_en': pcd_map_matching_en},YamlPath,{'yaml_config_path': YamlPath} ] 
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
          front_end_slam_node,
          start_rviz2_node,
        ]
    )


