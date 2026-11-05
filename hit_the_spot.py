import time
import numpy as np
from mbot_bridge.api import MBot

lady_robot = MBot()
speed = 0.3  # m/s
obstacle_dist = 0.15  # wall follow when an obstacle is this close toward the goal


def normalize_angle(angle):  # Avoid turning fully, choose shorter.
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


def hits_the_spot(goal, pose):
    """Hit the spot: P control on x, y, and theta at the same time."""
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    vx = np.clip(1.0 * ex, -speed, speed)
    vy = np.clip(1.0 * ey, -speed, speed)
    wz = np.clip(1.5 * normalize_angle(goal[2] - pose[2]), -1.5, 1.5)
    return vx, vy, wz


def main():
    # Reset the robot odometry to zero at the beginning of the run.
    lady_robot.reset_odometry()
    time.sleep(0.5)

    goal_x = float(input("Goal x: "))
    goal_y = float(input("Goal y: "))
    goal_theta = float(input("Goal theta (radians): "))
    goal = [goal_x, goal_y, goal_theta]

    try:
        while True:
            pose = lady_robot.read_odometry()
            dx = goal[0] - pose[0]
            dy = goal[1] - pose[1]
            dist = np.sqrt(dx**2 + dy**2)
            dtheta = abs(normalize_angle(goal[2] - pose[2]))

            # Done when within 3 cm and 0.15 rad (with a little margin)
            if dist < 0.03 and dthete < 0.15:
                break

            vx, vy, wz = hits_the_spot(goal, pose)
            lady_robot.drive(vx, vy, wz)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the robot before exiting.
        lady_robot.stop()

        x, y, theta = lady_robot.read_odometry()
        print("Final odometry: x =", round(x, 3), "y =", round(y, 3), "theta =", round(theta, 3))


if __name__ == "__main__":
    main()