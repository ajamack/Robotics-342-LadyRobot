import time
import numpy as np
from mbot_bridge.api import MBot

lady_robot = MBot()
speed = 0.3 # m/s
obstacle_dist = 0.4 # wall follow when an obstacle is this close toward the goal
wall_dist = 0.35 # distance to keep from wall while following it


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
 
 
def is_goal_angle_obstructed(ranges, thetas, goal, pose):
    """True if an obstacle is close in the direction of the goal."""
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    goal_angle = normalize_angle(np.arctan2(dy, dx) - pose[2])  # robot frame
    goal_dist = np.sqrt(dx**2 + dy**2)
 
    idx = find_min_nonzero_dist_in_slice(ranges, thetas, goal_angle, np.pi / 3)
    if idx == -1:
        return False
    # Obstructed if the obstacle is close AND in front of the goal
    return ranges[idx] < obstacle_dist and ranges[idx] < goal_dist
 
 
def drive_to_pose(goal, pose):
    """Hit the spot: P control on x, y, and theta at the same time."""
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    # Rotate the error into the robot's frame
    ex = np.cos(pose[2]) * dx + np.sin(pose[2]) * dy
    ey = -np.sin(pose[2]) * dx + np.cos(pose[2]) * dy
    vx = np.clip(1.0 * ex, -speed, speed)
    vy = np.clip(1.0 * ey, -speed, speed)
    wz = np.clip(1.5 * normalize_angle(goal[2] - pose[2]), -1.5, 1.5)
    return vx, vy, wz
 
 
def wall_follow(ranges, thetas):
    """Wall follower: move along the nearest wall while keeping wall_dist."""

    idx = find_min_nonzero_dist_in_slice(ranges, thetas, 0, 2 * np.pi)
    if idx == -1:
        return 0, 0, 0
    wall = np.array([np.cos(thetas[idx]), np.sin(thetas[idx]), 0])
    tangent = np.cross(wall, [0, 0, 1])
    correction = 1.5 * (ranges[idx] - wall_dist)
    vx = speed * tangent[0] + correction * wall[0]
    vy = speed * tangent[1] + correction * wall[1]
    return vx, vy, 0
 
# Reset the robot odometry to zero at the beginning of the run.
lady_robot.reset_odometry()
time.sleep(0.5)

goal_x = float(input("Goal x: "))
goal_y = float(input("Goal y: "))
goal_theta = float(input("Goal theta (radians): "))
goal = [goal_x, goal_y, goal_theta]

"""
TODO: (P2.3 & P2.4) Write code to drive to the given goal while avoiding
obstacles here.

HINT: You may reuse your code and util functions from wall follower and hit the
spot wherever you need to.
"""

while True:
    pose = lady_robot.read_odometry()
    dist_error = np.sqrt((goal[0] - pose[0])**2 + (goal[1] - pose[1])**2)
    theta_error = abs(normalize_angle(goal[2] - pose[2]))
 
    # Done when within 3 cm and 0.15 rad (with a little margin)
    if dist_error < 0.02 and theta_error < 0.1:
        break
 
    ranges, thetas = lady_robot.read_lidar()
 
    if is_goal_angle_obstructed(ranges, thetas, goal, pose):
        vx, vy, wz = wall_follow(ranges, thetas)
    else:
        vx, vy, wz = drive_to_pose(goal, pose)
 
    lady_robot.drive(vx, vy, wz)
    time.sleep(0.05)

# Stop the robot before exiting.
lady_robot.stop()

x, y, theta = lady_robot.read_odometry()
print("Final odometry: x =", round(x, 3), "y =", round(y, 3), "theta =", round(theta, 3))
 