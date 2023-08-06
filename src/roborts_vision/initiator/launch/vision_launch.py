from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
        ld = LaunchDescription()
        node_detect = Node(
            package="node_detect" , 
            executable="node_detect"
        )
        node_resolve = Node(
            package="node_resolve" ,
            executable="node_resolve"
        )
        node_predict = Node(
            package="node_predict" ,
            executable="node_predict"
        )
        node_usart = Node(
            package="roborts_center_usart" ,
            executable="roborts_center_usart"
        )
        ld.add_action(node_detect)
        ld.add_action(node_resolve)
        ld.add_action(node_predict)
        ld.add_action(node_usart)
        return ld