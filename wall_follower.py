import numpy as np
import time
from mbot_bridge.api import MBot

def find_min_dist(ranges, thetas):

    min_dist, min_angle = None, None

    # Convert lists into np arrays for manipulation
    ranges = np.array(ranges)
    thetas = np.array(thetas)
    valid_ind = (ranges > 0).nonzero()
    valid_ranges = ranges[valid_ind] 
    valid_thetas = thetas[valid_ind]
    min_ind = np.argmin(valid_ranges) # Closest object, smallest value
    min_dist = valid_ranges[min_ind]
    min_angle = valid_thetas[min_ind]

    return min_dist, min_angle

lady_robot = MBot()
setpoint = 0.3  # TODO: Pick your setpoint.
Kp = 0.2

try:
    while True:
        # Read the latest Lidar scan.
        ranges, thetas = lady_robot.read_lidar()

        # Get the distance and angle to the wall.
        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)

        error = dist_to_wall - setpoint # Find the error
        speed = Kp * error
        speed = np.clip(speed, -0.2, 0.2) # Can't go faster tham 0.3

        vx = speed * np.cos(angle_to_wall)
        vy = speed * np.sin(angle_to_wall)
    
        lady_robot.drive(vx, vy, 0)
        time.sleep(0.2)

        # TODO: Implement the 2D Follow Me controller
        # Hint: Look at your code from follow_1D
        # Hint: When you compute the velocity command, you might find the functions
        # np.sin(value) and np.cos(value) helpful!

        # Optionally, sleep for a bit before reading a new scan.
except KeyboardInterrupt:
    pass
    # Catch any exception, including the user quitting, and stop the robot.
finally:
    lady_robot.stop()
