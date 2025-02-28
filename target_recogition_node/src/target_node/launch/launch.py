from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_node',       
            executable='camera',
        ),
        Node(
            package='target_node',
            executable='track',
        ),
        Node(
            package='target_node',
            executable='predict',
        )
    ])