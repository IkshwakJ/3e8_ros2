import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ikshwak/3e8_v1/3e8_ros2/install/foxglove_bridge'
