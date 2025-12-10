import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Robotica-2026-1/proyecto_Robot1/Robot_1/install/my_robot_descripction'
