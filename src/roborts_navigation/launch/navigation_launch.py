from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    navigation_node = Node(
                                package="roborts_navigation",
                                executable='roborts_navigation',
                                output='screen',emulate_tty=True
                                )
    
    center_usart_node = Node(
                                package="roborts_center_usart",
                                executable='roborts_center_usart',
                                output='screen',emulate_tty=True
                                )
    ld.add_action(navigation_node)
    ld.add_action(center_usart_node)
    return ld


