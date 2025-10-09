import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntuuser/ros2_ws/src/turtlebot3_trailer/install/turtlebot3_trailer'
