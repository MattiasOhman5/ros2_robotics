from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your URDF/xacro
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_trailer'),
        'urdf',
        'trailer.urdf.xacro'
    )

    return LaunchDescription([
        # Launch Gazebo (Ignition/GZ)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        )
    ])

