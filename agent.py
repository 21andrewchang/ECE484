import numpy as np
import math
import carla
import csv
from simple_pid import PID  # Import PID controller
import matplotlib.pyplot as plt
from scipy.spatial import KDTree



class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle
        self.theta_ref = 0
        self.pre_waypoints = np.zeros(shape=(20,3)) # defines the initial 20 waypoints in x,y,z format
        self.pre_waypoints[0][0] = 96  # sets initial x
        self.pre_waypoints[0][1] = -110  # sets initial y
        self.L = 2.87  # Wheelbase of the vehicle
        self.csv_written = False  # Flag to track CSV write
        # PID Controller Parameters
        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.05
        self.pid_controller = PID(self.Kp, self.Ki, self.Kd, setpoint=0)
        self.pid_controller.output_limits = (-1, 1)  # Limit control signal between -1 and 1
        self.prev_speed_error = 0
        self.integrated_error = 0
        self.prev_control_signal = 0
        self.smoothed_control_signal = 0  # Initialize smoothed control signal

        self.csv_file = "control_values.csv"  # CSV file to store control values
        self.csv_header = ['Throttle', 'Brake', 'PID_Output']

        # Create CSV file and writer
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.csv_header)

        # Lists to store logged data
        self.throttle_values = []
        self.brake_values = []
        self.pid_values = []


        self.racing_x = []
        self.racing_y = []

        # Define the CSV file path for the racing line
        racing_line_csv = 'racing_line.csv'

        # Read racing line points from CSV
        with open(racing_line_csv, newline='') as file:
            reader = csv.reader(file)
            header = next(reader)  # Skip the header row
            for row in reader:
                x, y = map(float, row)
                self.racing_x.append(x)
                self.racing_y.append(y)

    
       
    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        control = carla.VehicleControl()


        #print('transform', transform)
        curr_x = transform.location.x
        curr_y = transform.location.y
        curr_yaw = math.radians(transform.rotation.yaw)
        heading_angles = self.calculate_heading_angles(waypoints)
        #heading_angles = self.calculate_heading_angles(boundary)
        expected_speed = self.classify_corners(heading_angles)
        target_speed = self.speed_controller(curr_x, curr_y, curr_yaw, filtered_obstacles, expected_speed)
        lookahead_point = self.get_lookahead_point(waypoints, 1)
        #target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, boundary)
        race_x = self.racing_x
        race_y = self.racing_y
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, race_x,race_y, boundary, filtered_obstacles)
        # print('Obstacles: ', filtered_obstacles)
        # for obstacle in filtered_obstacles:
        #     if obstacle:
        #         print('Obstacle position: ', obstacle.bounding_box.location)
        #print(['steering angle','target speed', 'yaw'])
        #print([targboundary
        #    print("WAYPOINT")

        #self.write_to_csv(boundary, waypoints, heading_angles)

        speed_error = target_speed - vel.length()
        self.integrated_error += speed_error
        control_signal = self.pid_controller(speed_error)


        # Apply derivative filtering to control signal
        derivative_signal = control_signal - self.prev_control_signal
        filtered_signal = 0.6 * self.prev_control_signal + 0.4 * derivative_signal  # Example low-pass filter
        self.prev_control_signal = control_signal

        #USED TO BE 0.8 0.2

        # Apply smoothing to filtered signal (example using simple moving average)
        smoothing_factor = 0.6  # Adjust smoothing factor as needed
        self.smoothed_control_signal = (
            smoothing_factor * self.smoothed_control_signal + (1 - smoothing_factor) * filtered_signal
        )

        # Apply control signal to throttle/brake
        if self.smoothed_control_signal > 0:
            control.throttle = 0.0
            control.brake = max(abs(self.smoothed_control_signal), 1.0)  # Brake control
        else:
            control.throttle = min(abs(self.smoothed_control_signal), 1)  # Throttle control
            control.brake = 0.0

        # Apply control signal to throttle/brake
        # if control_signal > 0:
        #     control.throttle = 0.0
        #     control.brake = min(abs(control_signal), 1.0)  # Brake control
        # else:
        #     control.throttle = min(abs(control_signal), 1)  # Throttle control
        #     control.brake = 0.0

        # Apply steering control
        control.steer = np.clip(target_steering, -2, 2)
        #self.extract_boundary_points(boundary, "points")
        self.write_to_csv(boundary, "boundary.csv")
        if not self.csv_written:
            self.written_to_csv(boundary, waypoints, heading_angles)
            #self.extract_boundary_points(boundary, "points")

            self.csv_written = True

        #print(f"Current Speed: {vel.length()}, Target Speed: {target_speed}")
        #print(f"Speed Error: {speed_error}, Control Signal: {control_signal}")
        #print(f"Throttle: {control.throttle}, Brake: {control.brake}")
        # Log data
        self.throttle_values.append(control.throttle)
        self.brake_values.append(control.brake)
        self.pid_values.append(control_signal)  # Assuming control_signal is the PID output

        # Write data to CSV
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([control.throttle, control.brake, control_signal])
        return control

    
    def calculate_heading_angles(self, waypoints):
        x_coordinates = [point[0] for point in waypoints]
        y_coordinates = [point[1] for point in waypoints]
        heading_angles = []
        for i in range(len(waypoints) - 1):
            dx = x_coordinates[i + 1] - x_coordinates[i]
            dy = y_coordinates[i + 1] - y_coordinates[i]
            heading_angle = np.arctan2(dy, dx)
            heading_angles.append(heading_angle)
        return heading_angles







    def classify_corners(self, heading_angles):
        max_speed = 30  # Maximum speed
        min_speed = 10  # Minimum speed
        max_angle_diff = 0.5  # Maximum angle difference for minimum speed

        for i in range(len(heading_angles) - 1):
            angle_diff = abs(heading_angles[i + 1] - heading_angles[i])
            speed_range = max_speed - min_speed
            speed_ratio = 1 - (angle_diff / max_angle_diff)  # Inverse relationship: higher angle diff = slower speed
            speed = min_speed + (speed_range * speed_ratio)
            if(speed < min_speed):
                return min_speed
            return speed  # Return speed for the first segment found
        return max_speed

    # def speed_controller( self, curr_x, curr_y, curr_yaw, filtered_obstacles, expected_speed):
    #     if filtered_obstacles:
    #         for obstacle in filtered_obstacles:
    #             loc = obstacle.get_location()
    #             obstacle_x = loc.x
    #             obstacle_y = loc.y
    #             print("obstacle type: ", obstacle.type_id)
    #             print('obstacle location',obstacle.get_location())
    #             distance = np.sqrt((curr_x - obstacle_x)**2 + (curr_y - obstacle_y)**2 )
    #             print('distance to obstacle: ', distance)
    #             if distance < 100:
    #                 print('under 100')
    #             elif distance < 50:
    #                 print('under 50')
    #                 return 1
    #             if "vehicle" not in obstacle.type_id:
    #                 print("not a car")
    #                 return 0

    #     return expected_speed  # Return target_speed if filtered_obstacles is empty

    def speed_controller(self, curr_x, curr_y, curr_yaw, filtered_obstacles, expected_speed, max_distance=30, min_distance=15):
        spawner_passed = False
        if filtered_obstacles:
            for obstacle in filtered_obstacles:
                print("Obstacle: ", obstacle)
                loc = obstacle.get_location()
                obstacle_x = loc.x
                obstacle_y = loc.y

                # Calculate distance
                distance = np.sqrt((curr_x - obstacle_x)**2 + (curr_y - obstacle_y)**2)
                
                # Calculate angle between agent's heading and obstacle
                angle_to_obstacle = np.arctan2(obstacle_y - curr_y, obstacle_x - curr_x)
                angle_diff = angle_to_obstacle - curr_yaw
                angle_diff = (np.degrees(angle_diff) + 180) % 360 - 180  # Convert to degrees [-180, 180]

                # Determine the general location of the obstacle using angles
                if angle_diff >= -20 and angle_diff <= 20:  # Front sector: -80 to 80 degrees
                    print('Obstacle in front or close to front.')
                else:
                    print('Obstacle behind or to the sides.')

                if "vehicle" not in obstacle.type_id:
                    if angle_diff >= -20 and angle_diff <= 20:
                        print("Pedestrian in front, emergency stopping.")
                        self.hand_brake = True
                        spawner_passed = True
                        return 0

                # Check if the obstacle is too close in front
                if distance < min_distance and (angle_diff >= -20 and angle_diff <= 20):
                    print('Too close to obstacle in front. Slowing down!')
                    return 3  # Stop or slow down

        return expected_speed  # Return target_speed if no obstacles or if they are not too close



    def get_lookahead_point(self, waypoints, distance):
        lookahead_index = min(len(waypoints) - 1, distance)
        return waypoints[lookahead_index]

    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, racing_x, racing_y, boundary, filtered_obstacles):
        L = 2.87  # Wheelbase of the vehicle
        left = boundary[0]
        right = boundary[1]
        print("Left boundaries: ", left)
        target = (left[6].transform.location + right[6].transform.location)/2
        if filtered_obstacles:
            for obstacle in filtered_obstacles:
                loc = obstacle.get_location()
                obstacle_x = loc.x
                obstacle_y = loc.y
                distance = np.sqrt((curr_x - obstacle_x)**2 + (curr_y - obstacle_y)**2)

                if distance <=20:
                    # Calculate distance to left boundary point
                    distance_to_left = np.sqrt((left[8].transform.location.x - obstacle_x)**2 + (left[8].transform.location.y - obstacle_y)**2)
                    
                    # Calculate distance to right boundary point
                    distance_to_right = np.sqrt((right[8].transform.location.x - obstacle_x)**2 + (right[8].transform.location.y - obstacle_y)**2)
                    
                    if distance_to_left < distance_to_right:
                        print(f"Obstacle is closer to the left boundary.")
                        # Take action based on left boundary proximity
                        target = (right[5].transform.location+target)/2
                    else:
                        print(f"Obstacle is closer to the right boundary.")
                        target = (left[5].transform.location+target)/2
                        # Take action based on right boundary proximity
                
        # with open("temp_bound.csv", mode='a', newline='') as file:  # Append mode
        #  
        #     writer = csv.writer(file)
        #     writer.writerow([target.x,target.y])
        dx = target.x-curr_x
        dy = target.y-curr_y
        alpha = math.atan2(dy,dx)
        euclid = math.sqrt(dx**2+dy**2)
        target_steering = math.atan(2*L*math.sin(alpha-curr_yaw)/euclid)
        return target_steering


    # def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, racing_x, racing_y, boundary):
    #     L = 2.87  # Wheelbase of the vehicle
    #     left = boundary[0]
    #     right = boundary[1]
    #     target = (left[5].transform.location + right[5].transform.location)/2

    #     temp_x, temp_y = self.find_closest_point(target.x, target.y, racing_x, racing_y)

    #     # Calculate alpha and euclidean distance to the target point
    #     dx = temp_x - curr_x
    #     dy = temp_y - curr_y
    #     alpha = math.atan2(dy, dx)
    #     euclid = math.sqrt(dx**2 + dy**2)

    #     # Calculate the target steering angle
    #     target_steering = math.atan(2 * L * math.sin(alpha - curr_yaw) / euclid)
    #     return target_steering


    def find_closest_point(self, target_x, target_y, racing_x, racing_y):
        min_dist = float('inf')
        closest_index = None

        for i in range(len(racing_x)):
            dx = racing_x[i] - target_x
            dy = racing_y[i] - target_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < min_dist:
                min_dist = dist
                closest_index = i

        return racing_x[closest_index], racing_y[closest_index]




    def written_to_csv(self, boundary, waypoints, heading_angles):
        csv_file = "data.csv"
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # writer.writerow('Boundary Left (x,y)')
            # writer.writerow([(point[0] for point in boundary[0])])
            # writer.writerow([(point[1] for point in boundary[0])])
            # writer.writerow('Boundary Right')
            # writer.writerow([(point[0] for point in boundary[1]),(point[1] for point in boundary[1])])
            writer.writerow(['Waypoint X', 'Waypoint Y'])
            writer.writerows(waypoints)
            writer.writerow(['Heading Angles'])
            writer.writerow(heading_angles)
        print(f"Data written to {csv_file}")
    
    def check_at_waypoint(self, curr_x, curr_y, waypoints, threshold=1.0):
        for waypoint in waypoints:
            wp_x, wp_y = waypoint[0], waypoint[1]
            distance = math.sqrt((curr_x - wp_x) ** 2 + (curr_y - wp_y) ** 2)
            if distance <= threshold:
                return True
        return False
    
    def write_to_csv(self, boundary, filename):
        with open(filename, mode='a', newline='') as file:  # Append mode
            writer = csv.writer(file)
            # Get the next left, right, and target points
            left_point = (boundary[0][5].transform.location.x, boundary[0][5].transform.location.y)
            right_point = (boundary[1][5].transform.location.x, boundary[1][5].transform.location.y)
            target_point = ((left_point[0] + right_point[0]) / 2, (left_point[1] + right_point[1]) / 2)
            
            # Write to CSV
            writer.writerow(['Left X', 'Left Y', 'Right X', 'Right Y', 'Target X', 'Target Y'])
            writer.writerow([left_point[0], left_point[1], right_point[0], right_point[1], target_point[0], target_point[1]])


    
    def write_points_to_csv(self, points, filename):
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y'])
            writer.writerows(points)
        print(f"Points written to {filename}")

    def plot_graphs(self):
        # Plot Throttle
        plt.figure(figsize=(10, 5))
        plt.plot(self.throttle_values)
        plt.xlabel('Time Step')
        plt.ylabel('Throttle')
        plt.title('Throttle Control')
        plt.grid(True)
        plt.show()

        # Plot Brake
        plt.figure(figsize=(10, 5))
        plt.plot(self.brake_values)
        plt.xlabel('Time Step')
        plt.ylabel('Brake')
        plt.title('Brake Control')
        plt.grid(True)
        plt.show()

        # Plot PID Values
        plt.figure(figsize=(10, 5))
        plt.plot(self.pid_values)
        plt.xlabel('Time Step')
        plt.ylabel('PID Output')
        plt.title('PID Control Output')
        plt.grid(True)
        plt.show()

    def extract_boundary_points(self, boundary, output_dir):
        left_boundary_points = []
        right_boundary_points = []
        midpoint_points = []

        left = boundary[0]
        right = boundary[1]

        for i in range(len(boundary[0])):  # Consider 10m ahead for heading angle calculation
            # Extract left and right boundary points
            left_point = (left[i].transform.location.x, left[i].transform.location.y)
            right_point = (right[i].transform.location.x, right[i].transform.location.y)
            
            left_boundary_points.append(left_point)
            right_boundary_points.append(right_point)

            # Calculate and extract midpoint points
            target = (left[i].transform.location + right[i].transform.location) / 2
            midpoint = ((left_point[0] + right_point[0]) / 2, (left_point[1] + right_point[1]) / 2)
            midpoint_points.append(midpoint)

        # Write points to CSV files
        self.write_points_to_csv(left_boundary_points, f"{output_dir}/left_boundary_points.csv")
        self.write_points_to_csv(right_boundary_points, f"{output_dir}/right_boundary_points.csv")
        self.write_points_to_csv(midpoint_points, f"{output_dir}/midpoint_points.csv")

        return left_boundary_points, right_boundary_points, midpoint_points