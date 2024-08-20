import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devin/Documents/Github-Projects/ROS2_Projects/ros2_ws/install/simple_pub_sub'
