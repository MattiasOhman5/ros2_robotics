from setuptools import find_packages, setup

package_name = 'lab2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_lab2.py'])
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
            'mpc_controller=lab2_pkg.mpc_controller:main',
            'mpc_controller_2=lab2_pkg.mpc_controller_2:main',
            'trajectory_1=lab2_pkg.trajectory_1:main',
            'trajectory_2=lab2_pkg.trajectory_2:main',
            'trajectory_circle=lab2_pkg.trajectory_circle:main',
            'mpc_controller_sim=lab2_pkg.mpc_controller_sim:main',
            'mpc_controller_2_sim=lab2_pkg.mpc_controller_2_sim:main',
        ],
    },
)
