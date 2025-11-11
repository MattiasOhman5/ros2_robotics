#!/home/ros2_ws/.venv/bin/python3
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch', 'rviz'),
         glob(os.path.join('launch', 'rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='name',
    maintainer_email='name@users.org',
    description='Exploration template package for R7021E',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_controller=mpc_pkg.mpc_controller:main',
            'mpc_controller_2=mpc_pkg.mpc_controller_2:main',
            'trajectory_1=mpc_pkg.trajectory_1:main',
            'trajectory_2=mpc_pkg.trajectory_2:main',
            'trajectory_circle=mpc_pkg.trajectory_circle:main',
            'mpc_controller_sim=mpc_pkg.mpc_controller_sim:main',
            'mpc_controller_2_sim=mpc_pkg.mpc_controller_2_sim:main',
            'visual_node=mpc_pkg.visualization_node:main'
        ],
    },
)
