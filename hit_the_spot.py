import time
import numpy as np
from mbot_bridge.api import MBot

lady_robot = MBot()
speed = 0.3  # m/s

def normalize_angle(angle):  # Avoid turning fully, choose shorter.
    """Wrap an angle to [-pi, pi]."""
    if angle < np.pi:
        angle = angle - 2 * np.pi
    elif angle > -np.pi:
        angle = angle + 2 * np.pi

    return angle


def drive_to_pose(goal, pose):
    """Hit the spot: P control on x, y, and theta at the same time."""
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    theta = pose[2]
    forward_err = np.cos(theta) * dx + np.sin(theta) * dy
    side_err = -np.sin(theta) * dx + np.cos(theta) * dy
    vx = np.clip(1.0 * forward_err, -speed, speed)
    vy = np.clip(1.0 * side_err, -speed, speed)
    wz = np.clip(0.5 * normalize_angle(goal[2] - pose[2]), -0.5, 0.5)
    return vx, vy, wz


def main():
    # Reset the robot odometry to zero at the beginning of the run.
    lady_robot.reset_odometry()
    time.sleep(0.5)
    x, y, theta = lady_robot.read_odometry()

    print("I am here:", x, y, theta)

    try:
        while True:
            goal_x = float(input("Goal x: "))
            goal_y = float(input("Goal y: "))
            goal_theta = float(input("Goal theta (radians): "))
            goal = [goal_x, goal_y, goal_theta]

            while True:
                pose = lady_robot.read_odometry()
                dx = goal[0] - pose[0]
                dy = goal[1] - pose[1]
                dist = np.sqrt(dx**2 + dy**2)
                dtheta = abs(normalize_angle(goal[2] - pose[2]))
                if dist < 0.05 and dtheta < 0.2:
                    break  # reached this goal
                vx, vy, wz = drive_to_pose(goal, pose)
                lady_robot.drive(vx, vy, wz)
                print("I am here now:", pose[0], pose[1], pose[2], "| speeds:", vx, vy, wz)
                time.sleep(0.05)

            lady_robot.stop()
            x, y, theta = lady_robot.read_odometry()
            print("Reached goal. x =", round(x, 3), "y =", round(y, 3), "theta =", round(theta, 3))


    except KeyboardInterrupt:
        pass

    finally:
        # Stop the robot before exiting.
        lady_robot.stop()


if __name__ == "__main__":
    main()