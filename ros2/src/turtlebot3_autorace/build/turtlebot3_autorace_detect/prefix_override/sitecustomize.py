import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/masters/ros2/src/turtlebot3_autorace/install/turtlebot3_autorace_detect'
