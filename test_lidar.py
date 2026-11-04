from mbot_bridge.api import MBot
lady_robot = MBot()
ranges, thetas = lady_robot.read_lidar()
print("got scan", len(ranges))