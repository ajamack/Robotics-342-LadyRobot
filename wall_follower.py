import time
import numpy as np
from mbot_bridge.api import MBot


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def find_min_nonzero_dist_in_slice(ranges, thetas, angle, slice_size):
    """Index of the shortest nonzero ray within slice_size/2 of angle (-1 if none)."""
    min_idx = -1
    for i in range(len(ranges)):
        in_slice = abs(normalize_angle(thetas[i] - angle)) <= slice_size / 2
        if in_slice and ranges[i] > 0:
            if min_idx == -1 or ranges[i] < ranges[min_idx]:
                min_idx = i
    return min_idx


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    idx = find_min_nonzero_dist_in_slice(ranges, thetas, 0, 2 * np.pi) # full circle
    if idx == -1:
        return None, None
    return ranges[idx], thetas[idx]


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


def wall_follow(ranges, thetas, setpoint, speed):
    """Move along the nearest wall while keeping setpoint distance from it."""
    dist, angle = find_min_dist(ranges, thetas)
    if dist is None:
        return 0, 0, 0
    wall = [np.cos(angle), np.sin(angle), 0] # points at wall
    tangent = cross_product(wall, [0, 0, 1]) # points along wall
    correction = 1.5 * (dist - setpoint) # Find the error
    vx = speed * tangent[0] + correction * wall[0]
    vy = speed * tangent[1] + correction * wall[1]
    return vx, vy, 0


lady_robot = MBot()
setpoint = 0.35 # m from wall
speed = 0.3 # m/s

try:
    while True:
        # Read the latest lidar scan.
        ranges, thetas = lady_robot.read_lidar()

        vx, vy, wz = wall_follow(ranges, thetas, setpoint, speed)
        lady_robot.drive(vx, vy, wz)

        time.sleep(0.1)
finally:
    lady_robot.stop()