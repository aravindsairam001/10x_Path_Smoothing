import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aero-ws/10x_Path_Smoothing/turtlebot_ws/install/turtlebot_path_smoothing'
