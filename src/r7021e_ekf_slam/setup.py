from setuptools import setup

package_name = 'r7021e_ekf_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/r7021e_ekf_slam']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/r7021e_ekf_slam.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Nikolaos Stathoulopoulos',
    maintainer_email='niksta@ltu.se',
    description='EKF-SLAM teaching package for ROS 2 Jazzy.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_slam_node = r7021e_ekf_slam.ekf_slam_node:main',
        ],
    },
)
