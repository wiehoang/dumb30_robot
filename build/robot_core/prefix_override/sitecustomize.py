import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wie/Project/Robotics/dumb30-robot/install/robot_core'
