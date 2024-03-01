import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.accel = []
        self.time = []
        self.log_acceleration = False

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    '''
    https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    Tasks 1: Read the documentation 
             Extract yaw, velocity, vehicle_position_x, vehicle_position_y
    Hint: you may use the the helper function(quaternion_to_euler()) we provide
          to convert from quaternion to euler
    '''
    def extract_vehicle_info(self, currentPose):
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        vel = self.prev_vel

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        q_x = currentPose.pose.orientation.x
        q_y = currentPose.pose.orientation.y
        q_z = currentPose.pose.orientation.z
        q_w =currentPose.pose.orientation.w
        euler = quaternion_to_euler(q_x, q_y, q_z, q_w) # [roll, pitch, yaw]
        yaw = euler[2]

        return pos_x, pos_y, vel, yaw # note that yaw is in radian
    
    
    '''
    Task 2: Longtitudal Controller
    Based on all unreached waypoints, and your current vehicle state, decide your velocity
    '''
    # def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
    #     #turn = self.pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, future_unreached_waypoints[1], future_unreached_waypoints)

    #     checkpoints = curr_vel
    #     straight = 12
    #     sharp_turn = 8
    #     medium_turn = 9
    #     upcoming_x, upcoming_y = [], []
    #     x_straight, y_straight = False, False
    #     # check future reachable points based on velocity
    #     for point in future_unreached_waypoints[:checkpoints]:
    #         x = point[0]
    #         y = point[1]
    #         upcoming_x.append(x)
    #         upcoming_y.append(y)
    #     # checks if all future x/y points are same
    #     if len(upcoming_x) > 0:
    #         x_straight = upcoming_x.count(upcoming_x[0]) == len(upcoming_x)
    #     if len(upcoming_y) > 0:
    #         y_straight = upcoming_x.count(upcoming_y[0]) == len(upcoming_y)
    #     # return velocity
    #     if x_straight or y_straight:
    #         return straight
        
    #     return sharp_turn
    
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
        straight_speed = 13
        slow_corner_speed = 8
        medium_corner_speed = 11
        smedium_corner_speed = 9
        
        # Calculate heading angles for the upcoming segments
        heading_angles = []
        for i in range(len(future_unreached_waypoints) - 1):
            dx = future_unreached_waypoints[i+1][0] - future_unreached_waypoints[i][0]
            dy = future_unreached_waypoints[i+1][1] - future_unreached_waypoints[i][1]
            heading_angle_i = np.arctan2(dy, dx)
            heading_angles.append(heading_angle_i)
        
        # Check if the upcoming segment is straight, a medium corner, or a slow corner based on the difference in heading angles
        for i in range(len(heading_angles) - 2):
            angle_diff = np.abs(heading_angles[i+2] - heading_angles[i])
            print('angledif:', angle_diff)
            if (angle_diff < 0.30 and angle_diff > 0.1):  # Adjust this threshold as needed for medium corners
                print(medium_corner_speed)
                return medium_corner_speed
            elif (angle_diff >= 0.30 and angle_diff < 0.45):  # Adjust this threshold as needed for medium corners
                print(smedium_corner_speed)
                return smedium_corner_speed
            elif angle_diff >= 0.45:  # Adjust this threshold as needed for medium corners
                print(slow_corner_speed)
                return slow_corner_speed
            else:
                # Return slow speed if a corner is detected
                print(straight_speed)
                return straight_speed
        return 13

        

    '''
    Task 3: Lateral Controller (Pure Pursuit)
    '''
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        wheelbase = 2 * self.L
        #ang_v_x = GetModelState().twist.angular.x
        #ang_v_y = GetModelState().twist.angular.y
        la_point_x = target_point[0]
        la_point_y = target_point[1]
        #dot_product = ang_v_x
        #print('lookahead point:', future_unreached_waypoints[4])
        ld = distance = math.sqrt((la_point_x - curr_x)**2 + (la_point_y - curr_y)**2)
        dx = la_point_x - curr_x
        dy = la_point_y - curr_y
        alpha = math.atan2(la_point_y-curr_y, la_point_x-curr_x) - curr_yaw
        sin_alpha = math.sin(alpha)
        target_steering = math.atan((wheelbase * sin_alpha)/(ld))
        #print(target_steering)

        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None
        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)
        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            if abs(acceleration) > 5:
                self.accel_ct += 1
            self.accel.append(acceleration)
            if len(self.time) == 0:
                self.time.append(1/100.0)
            else:
                self.time.append(self.time[-1] + 1/100.0)
        self.prev_vel = curr_vel

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)

        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
