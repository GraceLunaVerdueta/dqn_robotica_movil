import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/grace/Documents/Diplomado_Robotica/16_01_ws/install/labo_dqn'
