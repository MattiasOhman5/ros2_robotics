from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='lab1_pkg',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
        ),
    ])
