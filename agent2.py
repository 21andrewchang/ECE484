
import carla
import time
import numpy as np
import math

import matplotlib.pyplot as plt
import numpy as np
import csv

class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle
        self.theta_ref = 0
        self.pre_waypoints = np.zeros(shape=(20,3)) #defines the initial 20 waypoints in x,y,z format
        self.pre_waypoints[0][0]=115 #sets initial x
        self.pre_waypoints[0][1]=-100 #sets initial y

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):

        # Actions to take during each simulation step
        # Feel Free to use carla API; however, since we already provide info to you, using API will only add to your delay time
        # Currently the timeout is set to 10s

        print('boundaries: ', boundary)
        x_ref = waypoints[0][0]
        y_ref = waypoints[0][1]
        print("XREF ",x_ref)
        print("YREF ",y_ref)
        x_b = transform.location.x
        y_b = transform.location.y

        x_boundary = boundary[0][0]
        y_boundary = boundary[0][1]


        x_coordinates = [point[0] for point in waypoints]
        y_coordinates = [point[1] for point in waypoints]

        # Plot waypoints
        plt.figure(figsize=(8, 6))
        plt.plot(x_coordinates, y_coordinates, marker='o', color='blue', linestyle='-')
        plt.title('Waypoints')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.axis('equal')  # Equal aspect ratio
        #plt.show()

        # Write waypoints to CSV file
        csv_file = "waypoints.csv"
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y'])  # Write header
            for point in waypoints:
                writer.writerow(point)
        print(f"Waypoints written to {csv_file}")

        #self.vehicle.x
        #calculated angle based on x/y position differences in waypoints
        #use pre_waypoints to store previous waypoints defined in init section
        theta_ref = np.arctan2(-self.pre_waypoints[0][1] + waypoints[0][1], -self.pre_waypoints[0][0] + waypoints[0][0])
        self.pre_waypoints = waypoints

        theta_b = transform.rotation.yaw
        #Cars heading

        delta_x = math.cos(theta_ref) * (x_ref - x_b) + math.sin(theta_ref) * (y_ref - y_b)
        delta_y = -math.sin(theta_ref) * (x_ref - x_b) + math.cos(theta_ref) * (y_ref - y_b)
        delta_theta = theta_ref - theta_b
        print("DTHETA ",delta_theta)

        #calculation of steering angle

        control = carla.VehicleControl()
        control.throttle = 0.3  # Constant throttle
        control.brake = 0.0  # No braking
        control.steer = np.clip(delta_theta, -1, 1)  # Steer towards the waypoint's lateral position
        #clip keeps the steering angle in the correct bound to avoid errors with numpy
        return control


    # def classify_corners(self, heading_angles):
    #     straight_speed = 30
    #     slow_corner_speed = 8
    #     medium_corner_speed = 14
    #     smedium_corner_speed = 9
        
    #     for i in range(len(heading_angles) - 2):
    #         angle_diff = np.abs(heading_angles[i + 1] - heading_angles[i])
    #         if 0.1 < angle_diff < 0.30:
    #             return medium_corner_speed
    #         elif 0.30 <= angle_diff < 0.45:
    #             return smedium_corner_speed
    #         elif angle_diff >= 0.45:
    #             return slow_corner_speed
    #     return straight_speed


    # def calculate_heading_angles(self, boundary):
    #     heading_angles = []
    #     left = boundary[0]
    #     right = boundary[1]
    #     for i in range(len(boundary[0]) - 10): #10m ahead for heading angle calculation
    #         target = (left[i+10].transform.location + right[i+10].transform.location)/2
    #         current = (left[i].transform.location + right[i].transform.location)/2

    #         dx = target.x-current.x
    #         dy = target.y-current.y
    #         heading_angle = np.arctan2(dy, dx)
    #         heading_angles.append(heading_angle)

    #     return heading_angles

        
    
    # def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, lookahead_point):
    #     L = 2.5  # Wheelbase of the vehicle
    #     la_point_x, la_point_y = lookahead_point[0], lookahead_point[1]
    #     ld = math.sqrt((la_point_x - curr_x) ** 2 + (la_point_y - curr_y) ** 2)
    #     alpha = math.atan2(la_point_y - curr_y, la_point_x - curr_x) - curr_yaw
    #     target_steering = math.atan((2 * L * math.sin(alpha)) / ld)
    #     return target_steering