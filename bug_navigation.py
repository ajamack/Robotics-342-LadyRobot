import time
import numpy as np
from mbot_bridge.api import MBot

lady_robot = MBot()
speed = 0.3 # m/s
obstacle_dist = 0.4 # wall follow when an obstacle is this close toward the goal
wall_dist = 0.35 # distance to keep from wall while following it


def normalize_angle(angle):  # Avoid turning fully, choose shorter.
    """Wrap an angle to [-pi, pi]."""
    if angle > np.pi:
        angle = angle - 2 * np.pi
    elif angle < -np.pi:
        angle = angle + 2 * np.pi

    return angle


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


def drive_to_pose(goal, pose):
    """Hit the spot: P control on x, y, and theta at the same time."""
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    # Rotate the error into the robot's frame
    theta = pose[2]
    forward_err = np.cos(theta) * dx + np.sin(theta) * dy
    side_err = -np.sin(theta) * dx + np.cos(theta) * dy
    vx = np.clip(1.0 * forward_err, -speed, speed)
    vy = np.clip(1.0 * side_err, -speed, speed)
    wz = np.clip(1.5 * normalize_angle(goal[2] - pose[2]), -1.5, 1.5)
    return vx, vy, wz


def main():
    lady_robot.reset_odometry()
    time.sleep(0.5)

    # Define the goal
    goal_x = float(input("Goal x: "))
    goal_y = float(input("Goal y: "))
    goal_theta = float(input("Goal theta (radians): "))
    goal = [goal_x, goal_y, goal_theta]
    following = False

    try:
        while True:
            pose = lady_robot.read_odometry()
            ranges, thetas = lady_robot.read_lidar()

            # How far is the goal and which way is it?
            dx = goal[0] - pose[0]
            dy = goal[1] - pose[1]
            goal_dist = np.sqrt(dx**2 + dy**2)
            goal_angle = normalize_angle(np.arctan2(dy, dx) - pose[2])
            theta_error = abs(normalize_angle(goal[2] - pose[2]))
            print("Angular error:", normalize_angle(goal[2] - pose[2]))
            # Check 1: are we at the goal?
            if goal_dist < 0.03 and theta_error < 0.15:
                print("Goal reached!")
                break

            # Check 2: is something blocking the way to the goal?
            idx = find_min_nonzero_dist_in_slice(ranges, thetas, goal_angle, np.pi / 3)
            blocked = idx != -1 and ranges[idx] < obstacle_dist and ranges[idx] < goal_dist
            clear = idx == -1 or ranges[idx] > obstacle_dist + 0.3 or ranges[idx] > goal_dist

            if blocked:
                following = True
            elif clear:
                following = False

            if following:
                vx, vy, wz = wall_follow(ranges, thetas, wall_dist, speed)
            else:
                vx, vy, wz = drive_to_pose(goal, pose)

            lady_robot.drive(vx, vy, wz)
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print("Error:", e)

    finally:
        lady_robot.stop()

    x, y, theta = lady_robot.read_odometry()
    print("Final odometry: x =", round(x, 3), "y =", round(y, 3), "theta =", round(theta, 3))


main()