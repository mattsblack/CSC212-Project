import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matt/CSC212-Roomba-Project/robot_ws/src/install/create3_mapper'
