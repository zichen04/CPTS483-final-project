import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zich/Desktop/CPTS483-final-project/install/arm_teleop'
