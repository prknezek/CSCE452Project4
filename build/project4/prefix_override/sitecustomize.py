import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/grant/ros2_ws/src/CSCE435Project4/install/project4'
