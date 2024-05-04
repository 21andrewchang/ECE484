import numpy as np
import math
import carla

class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle
        self.theta_ref = 0
        self.pre_waypoints = np.zeros(shape=(20,3)) # defines the initial 20 waypoints in x,y,z format
        self.pre_waypoints[0][0] = 96  # sets initial x
        self.pre_waypoints[0][1] = -110  # sets initial y
        self.L = 2.5  # Wheelbase of the vehicle

    def run_step(self, filtered_obstacles, waypoints, transform, vel, boundary):
        print('transform', transform)
        curr_x = transform.location.x
        curr_y = transform.location.y
        curr_yaw = math.radians(transform.rotation.yaw)

        heading_angles = self.calculate_heading_angles(waypoints)
        target_speed = self.classify_corners(heading_angles)
        lookahead_point = self.get_lookahead_point(waypoints, distance=5)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, lookahead_point)

        control = carla.VehicleControl()
        control.throttle = np.clip((target_speed - self.vehicle.get_velocity().length()) / target_speed, 0, 1)
        control.steer = np.clip(target_steering, -1, 1)
        control.brake = 0.0

        return control

def calculate_heading_angles(waypoints):
    heading_angles = []
    for i in range(len(waypoints) - 1):
        dx = waypoints[i + 1][0] - waypoints[i][0]
        dy = waypoints[i + 1][1] - waypoints[i][1]
        heading_angle = np.arctan2(dy, dx)
        heading_angles.append(heading_angle)
    return heading_angles

def classify_corners(heading_angles):
    straight_speed = 13
    slow_corner_speed = 8
    medium_corner_speed = 11
    smedium_corner_speed = 9
    
    for i in range(len(heading_angles) - 2):
        angle_diff = np.abs(heading_angles[i + 2] - heading_angles[i])
        if 0.1 < angle_diff < 0.30:
            return medium_corner_speed
        elif 0.30 <= angle_diff < 0.45:
            return smedium_corner_speed
        elif angle_diff >= 0.45:
            return slow_corner_speed
    return straight_speed

def get_lookahead_point(waypoints, distance=5):
    lookahead_index = min(len(waypoints) - 1, distance)
    return waypoints[lookahead_index]

def pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, lookahead_point):
    L = 2.5  # Wheelbase of the vehicle
    la_point_x, la_point_y = lookahead_point[0], lookahead_point[1]
    ld = math.sqrt((la_point_x - curr_x) ** 2 + (la_point_y - curr_y) ** 2)
    alpha = math.atan2(la_point_y - curr_y, la_point_x - curr_x) - curr_yaw
    target_steering = math.atan((2 * L * math.sin(alpha)) / ld)
    return target_steering
