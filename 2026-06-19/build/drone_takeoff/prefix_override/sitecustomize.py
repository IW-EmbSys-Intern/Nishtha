import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/indowings/scripts/2026-06-19/install/drone_takeoff'
