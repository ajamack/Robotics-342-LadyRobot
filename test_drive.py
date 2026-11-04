import time
from mbot_bridge.api import MBot
lady_robot = MBot()
lady_robot.drive(0.1, 0, 0)
time.sleep(1)
lady_robot.stop()
print("drive done")
