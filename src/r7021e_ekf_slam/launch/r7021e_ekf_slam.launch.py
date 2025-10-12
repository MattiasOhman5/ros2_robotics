# r7021e_ekf_slam/launch/r7021e_ekf_slam.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('r7021e_ekf_slam')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with node parameters.'
        ),
        Node(
            package='r7021e_ekf_slam',
            executable='ekf_slam_node',
            name='ekf_slam_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
