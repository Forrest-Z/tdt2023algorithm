from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    """launch内容描述函数，由ros2 launch 扫描调用"""
    robots_perception = Node(
        package="robots_perception",
        executable="robots_perception",
                    emulate_tty=True,
                    output='screen',
    )
    # start_cam_cmd = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('usb_cam'), 'launch', 'demo_launch.py')])
                
    #          )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([robots_perception])
    
    
    # 返回让ROS2根据launch描述执行节点
    return launch_description