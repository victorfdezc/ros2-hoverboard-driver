from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2-hoverboard-driver',
            namespace='hello_world',
            executable='hello_world',
            name='hello_world'
        )
    ])