import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Robotica-2026-1/robot_xyz_ws/install/robot_xyz_bringup'
