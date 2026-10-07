import time
import numpy as np
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None

    for r, t in zip(ranges, thetas):
        if r <= 0:
            continue  # skip invalid rays
        if min_dist is None or r < min_dist:
            min_dist = r
            min_angle = t

    return min_dist, min_angle


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.zeros(3)
    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]
    return res


lady_robot = MBot()
setpoint = 0.3   # desired distance from the wall (m)
speed = 0.2      # speed along the wall (m/s)
Kp = 1.0         # gain for correcting distance to the wall

try:
    while True:
        # Read the latest lidar scan.
        ranges, thetas = lady_robot.read_lidar()

        dist, angle = find_min_dist(ranges, thetas)
        if dist is None:
            lady_robot.stop()
            time.sleep(0.1)
            continue

        # Unit vector pointing toward the wall
        to_wall = [np.cos(angle), np.sin(angle), 0]

        # Vector parallel to the wall (perpendicular to to_wall)
        along_wall = cross_product(to_wall, [0, 0, 1])

        # Drive along the wall, and correct toward/away to hold the setpoint
        error = dist - setpoint
        vx = speed * along_wall[0] + Kp * error * to_wall[0]
        vy = speed * along_wall[1] + Kp * error * to_wall[1]

        vx = np.clip(vx, -0.3, 0.3)
        vy = np.clip(vy, -0.3, 0.3)

        lady_robot.drive(vx, vy, 0)

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except:
    # Catch any exception, including the user quitting, and stop the robot.
    lady_robot.stop()