import rospy
import math
import argparse

from gazebo_msgs.msg import  ModelState
from controller import bicycleModel
import time

from lidarProcessing import LidarProcessing
from positionDetector import PositionDetector
from safetyDetector import SafetyDetector

import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

def getStoppingPoint(carPos, pedPos, stopDistance=1):
    carPos = np.array([carPos.pose.position.x, carPos.pose.position.y])

    unit_vector = carPos - pedPos
    unit_vector = unit_vector / np.linalg.norm(unit_vector)

    stopPos = pedPos + stopDistance*unit_vector

    return stopPos


def run_model(d_safe = 15,v_0 = 10, a_b = 5, t_react = 0):
    resolution = 0.1
    side_range = (-10, 10)
    fwd_range = (0., 25.)
    height_range = (-1.5, 0.5)

    # init rospy node
    rospy.init_node("model_dynamics")

    model = bicycleModel(velocity = v_0, deceleration = a_b)

    lidar = LidarProcessing(resolution=resolution, side_range=side_range, fwd_range=fwd_range,
                         height_range=height_range)
    posDetector = PositionDetector(resolution=resolution)
    safety = SafetyDetector(d_safe, resolution)

    endList = 0
    # the fixed path of the car
    pos_list = [[100,0], [300,0], [303.67,1.84], [305.5,5.51], [305.5,36.33],
                [303.67,40], [300,41.84], [-60,41.84], [-63.67,40], [-65.5,36.33],
                [-65.5,5.51], [-63.67,1.84], [-60,0],[-40,0],[-20,0],[0, 0]]
    pos_idx = 0

    targetState = ModelState()
    targetState.pose.position.x = pos_list[pos_idx][0]
    targetState.pose.position.y = pos_list[pos_idx][1]

    stopState = ModelState()

    rate = rospy.Rate(100)  # 100 Hz
    lidar.processLidar()

    pre_safe = True
    brake = False
    start_brake = False

    unsafe_time = -0.01
    
    # record the position of AV and pedestrain
    t = [0]
    x1 = []
    x2 = []
    d = []
    
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  model.getModelState()
        if not currState.success:
            continue

        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)

        # update birds eye view
        lidar.processLidar()

        # detect the position of the pedestrian in the image
        pedImgPosition = posDetector.getPosition()
        # Compute the distance between pedestrian and vehicle. Check if vehilce is within safety distance from the pedestrian
        safe, pedPosition, distance = safety.checkSafety(currState, pedImgPosition)

        t.append(t[-1] + 0.01)
        x1.append(currState.pose.position.x)
        x2.append(pedPosition[0] if pedPosition != 0 else 0)
        d.append(distance if distance != None else 0) 

        # Print current position of the vehicle
        # print(safe,distance,currState.pose.position.x,currState.pose.position.y)
        
        # If not safe start count down reaction time
        if not safe:
            if unsafe_time == -0.01:
                text = 'Distance to pedestrian is ' + str(distance) + '<=d_safe'
                print(text)
            unsafe_time += 0.01
        
        # If exceed reaction time, start braking
        if unsafe_time >= t_react:
            brake = True

        if not brake:
            if (distToTargetX < 1 and distToTargetY < 1):
                # if safe and at the target - move to the next target
                pos_idx = pos_idx+1
                pos_idx = pos_idx % len(pos_list)
                targetState = ModelState()
                targetState.pose.position.x = pos_list[pos_idx][0]
                targetState.pose.position.y = pos_list[pos_idx][1]
            else:
                # safe, but not yet at traget - chance control to move to target
                model.setModelState(currState, targetState, "run")
        else:
            # Braking the vehicle
            if start_brake == False:
                text = 'Start Braking, distance to pedestrian is ' + str(distance)
                print(text)
                start_brake = True
            model.setModelState(currState, targetState, "brake")

            if model.stopped:
                rospy.signal_shutdown('Vehicle Stopped')
                

    # Problem 5 plot x1, x2, d respect to t
    vars5 = f"dsense-{d_sense}_v0-{v_0}_ab-{a_b}_treact-{t_react}"
    problem_saved_dir = f'{os.getcwd()}/problem-answer'
    if not os.path.isdir(problem_saved_dir):
        os.mkdir(problem_saved_dir)
    plt.plot(t[1:], x1, label='x1')
    plt.plot(t[1:], x2, label='x2')
    plt.title(f'Problem 5')
    plt.xlabel('Time t (sec)')
    plt.ylabel('Variables')
    plt.legend()
    plt.savefig(f"{problem_saved_dir}/problem_5_{vars5}.png")
    
    # Problem 6 plot final d(t)
    vars6 = f"dsense-{d_sense}_ab-{a_b}_treact-{t_react}"
    problem_6_path = f"{problem_saved_dir}/problem_6_{vars6}.xlsx"
    last_d_idx = np.where(np.array(d) != 0)[0][-1]
    last_d = np.array(d)[last_d_idx]
    data = None
    if not os.path.isfile(problem_6_path):
        data = np.array([[v_0, last_d]])
    else:
        data = pd.read_excel(problem_6_path).to_numpy()
        data = np.append(data, np.array([[v_0, last_d]]), axis=0)
        data = np.sort(data, axis=0)
    pd.DataFrame(data, columns=['v_0', 'd(t)']).to_excel(problem_6_path, index=False)
    plt.cla()
    plt.plot(data[:, 0], data[:, 1], 'ro')
    plt.plot(data[:, 0], data[:, 1], 'k--')
    plt.title('Problem 6')
    plt.xlabel('v_0')
    plt.ylabel('d(t)')
    plt.savefig(f"{problem_saved_dir}/problem_6_{vars6}.png")

    rospy.spin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Running vehicle with pedestrain detection')

    d_sense_default = 15
    v_0_default = 5
    a_b_default = 5
    t_react_default = 0

    parser.add_argument('--d_sense', type = float, help = 'Safety distance that the vehicle start to brake.', default = d_sense_default)
    parser.add_argument('--v_0', type = float, help = 'Initial velocity of the vehicle.', default = v_0_default)
    parser.add_argument('--a_b', type = float, help = 'Deceleration rate of the vehicle.', default = a_b_default)
    parser.add_argument('--t_react', type = float, help = 'Reaction time of the vehicle.', default = t_react_default)

    argv = parser.parse_args()

    d_sense = argv.d_sense
    v_0 = argv.v_0
    a_b = argv.a_b
    t_react = argv.t_react

    run_model(d_safe = d_sense, v_0 = v_0, a_b = a_b, t_react = t_react)