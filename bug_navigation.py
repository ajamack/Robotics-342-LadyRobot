import time
import numpy as np
from mbot_bridge.api import MBot

lady_robot = MBot()
speed = 0.3 # m/s
obstacle_dist = 0.4 # wall follow when an obstacle is this close toward the goal
wall_dist = 0.35 # distance to keep from wall while following it


def normalize_angle(angle): # Avoid turning fully, choose shorter.
    """Wrap an angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def find_min_ray_in_slice(ranges, thetas, angle, slice_size):
    """Index of the shortest nonzero ray within slice_size/2 of angle (-1 if none)."""
    min_idx = -1
    for i in range(len(ranges)):
        in_slice = abs(normalize_angle(thetas[i] - angle)) <= slice_size / 2
        if in_slice and ranges[i] > 0:
            if min_idx == -1 or ranges[i] < ranges[min_idx]:
                min_idx = i
    return min_idx

def find_min_dist_all(ranges, thetas):
    return find_min_ray_in_slice(ranges, thetas, 0, 2 * np.pi)

def cross_product(v1, v2):
    """Cross product of two 3D vectors."""
    x = v1[1] * v2[2] - v1[2] * v2[1]
    y = v1[2] * v2[0] - v1[0] * v2[2]
    z = v1[0] * v2[1] - v1[1] * v2[0]
    return [x, y, z]

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


def wall_follow(ranges, thetas):
    """Slide along the closest wall while staying wall_dist away."""
    idx = find_min_dist_all(ranges, thetas)
    if idx == -1:
        return 0, 0, 0
    wall = [np.cos(thetas[idx]), np.sin(thetas[idx]), 0]
    along = cross_product(wall, [0, 0, 1])                # arrow pointing along the wall
    error = ranges[idx] - wall_dist                       # too far = positive, too close = negative
    vx = np.clip(speed * along[0] + error * wall[0], -speed, speed)
    vy = np.clip(speed * along[1] + error * wall[1], -speed, speed)
    return vx, vy, 0

def main():
    lady_robot.reset_odometry()
    time.sleep(0.5)

    # Define the goal
    goal_x = float(input("Goal x: "))
    goal_y = float(input("Goal y: "))
    goal_theta = float(input("Goal theta (radians): "))
    goal = [goal_x, goal_y, goal_theta]

    # Current state: "goal" or "wall"
    state = "goal"
    hit_dist = 0  # how far the goal was when we hit the wall

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

            # Check 1: are we at the goal?
            if goal_dist < 0.03 and theta_error < 0.15:
                print("Goal reached!")
                break

            # Check 2: is something blocking the way to the goal?
            idx = find_min_ray_in_slice(ranges, thetas, goal_angle, np.pi / 3)
            blocked = idx != -1 and ranges[idx] < obstacle_dist and ranges[idx] < goal_dist

            # Check 3: should we switch states?
            if state == "goal" and blocked:
                state = "wall"
                hit_dist = goal_dist
            elif state == "wall" and not blocked and goal_dist < hit_dist:
                state = "goal"

            # Move based on the state
            if state == "goal":
                vx, vy, wz = drive_to_pose(goal, pose)
            else:
                vx, vy, wz = wall_follow(ranges, thetas)

            lady_robot.drive(vx, vy, wz)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        lady_robot.stop()

    x, y, theta = lady_robot.read_odometry()
    print("Final odometry: x =", round(x, 3), "y =", round(y, 3), "theta =", round(theta, 3))


main()