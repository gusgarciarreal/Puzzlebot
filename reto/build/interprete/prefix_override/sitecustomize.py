import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/snorlix/ROS2/Puzzlebot/reto/install/interprete'
