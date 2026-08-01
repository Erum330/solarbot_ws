import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/erum_ifti/solarbot_ws-1/src/install/solarbot_safety'
