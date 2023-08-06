from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('roborts_navigation'), 'launch', 'navigation_launch.py')])
                
             )
    
    perception = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robots_perception'), 'launch', 'robots_perception.launch.py')])
                
             )
    
    ld.add_action(navigation)
    ld.add_action(perception)
    return ld


