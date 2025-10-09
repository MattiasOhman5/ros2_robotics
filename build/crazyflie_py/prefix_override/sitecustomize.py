import sys
if sys.prefix == '/home/ubuntuuser/ros2_ws/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntuuser/ros2_ws/install/crazyflie_py'
