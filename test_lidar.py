import time
from mbot_bridge.api import MBot

lady_robot = MBot()
lady_robot.reset_odometry()
time.sleep(0.5)

lady_robot.drive(0.5, 0, 0)
time.sleep(2)
lady_robot.stop()
print("After forward:", lady_robot.read_odometry())

lady_robot.drive(0, 0, 0.5)
time.sleep(2)
lady_robot.stop()
print("After turn:", lady_robot.read_odometry())