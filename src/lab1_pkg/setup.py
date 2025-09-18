from setuptools import find_packages, setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_lab1.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntuuser',
    maintainer_email='ubuntuuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node=lab1_pkg.simple_controller:main',
            'trajectory_node=lab1_pkg.trajectory:main'
        ],
    },
)
