#!/home/ros2_ws/.venv/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc_pkg',
            executable='mpc_controller_sim',
            name='mpc_controller',
            output='screen',
        ),
        Node(
            package='mpc_pkg',
            executable='trajectory_circle',
            name='trajectory_circle',
            output='screen',
        ),
        Node(
            package='mpc_pkg',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
        )
    ])
