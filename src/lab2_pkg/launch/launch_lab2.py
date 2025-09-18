from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab2_pkg',
            executable='mpc_controller',
            name='mpc_controller',
            output='screen',
        ),
        Node(
            package='lab2_pkg',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
        ),
    ])
