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
    d_actual = []
    d_sensor = []


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
        d_actual.append(60 - currState.pose.position.x) 
        d_sensor.append(pedPosition[0] - currState.pose.position.x if pedPosition != 0 else 0) 
        
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
                
    problem_saved_dir = f'{os.getcwd()}/problem-answer'
    if not os.path.isdir(problem_saved_dir):
        os.makedirs(problem_saved_dir)
    if not os.path.isdir(f"{problem_saved_dir}/trajectory"):
        os.makedirs(f"{problem_saved_dir}/trajectory")
        
            
    # Problem 5 plot x1, x2, d respect to t
    name = f"dsense-{d_sense}_v0-{v_0}_ab-{a_b}_treact-{t_react}"
    plt.plot(t[1:], x1, label='x1')
    plt.plot(t[1:], x2, label='x2')
    plt.title(f'Simulation Trajectory\n{name}')
    plt.xlabel('Time t (sec)')
    plt.ylabel('Variables')
    plt.legend()
    plt.savefig(f"{problem_saved_dir}/trajectory/{name}.png")
    

    # Problem 6 plot final d(t)   
    # problem_6_path = f"{problem_saved_dir}/problem_6.xlsx"
    # last_d = d_actual[-1]
    # data = None
    # if not os.path.isfile(problem_6_path):
    #     data = np.array([[v_0, last_d, d_sense, a_b, t_react]])
    # else:
    #     data = pd.read_excel(problem_6_path).to_numpy()
    #     data = np.append(data, np.array([[v_0, last_d, d_sense, a_b, t_react]]), axis=0)
    #     data_sorted_idx = np.argsort(data[:, 0])
    #     data = data[data_sorted_idx]
    # pd.DataFrame(data, columns=['v_0', 'd(t)', 'd_sense', 'a_b', 't_react']).to_excel(problem_6_path, index=False)
    # all_d_sense = list(set(data[:, 2]))
    # all_d_sense.sort()
    # color = {0:'r', 1:'b', 2:'g', 3:'m'}
    # plt.cla()
    # for i, j in enumerate(all_d_sense):
    #     data_partitioned = data[np.where(data[:, 2] == j)[0], :]
    #     plt.plot(data_partitioned[:, 0], data_partitioned[:, 1], f'{color[i]}o', label=f'd_sense={j}, a_b={a_b}, t_react={t_react}')
    #     plt.plot(data_partitioned[:, 0], data_partitioned[:, 1], f'{color[i]}--')
    # plt.axhline(y=0, color='k', linestyle='--')
    # plt.title(f'Comparison of d(t)\na_b={a_b} t_react={t_react}"')
    # plt.xlabel('v_0')
    # plt.ylabel('d(t) = x2(t) - x1(t)')
    # plt.xlim(0, 24)
    # plt.legend()
    # plt.savefig(f"{problem_saved_dir}/problem_6.png")


    # Problem 7 plot final d(t)   
    problem_7_path = f"{problem_saved_dir}/problem_7.xlsx"
    last_d = d_actual[-1]
    data = None
    if not os.path.isfile(problem_7_path):
        data = np.array([[v_0, last_d, d_sense, a_b, t_react]])
    else:
        data = pd.read_excel(problem_7_path).to_numpy()
        # data = np.append(data, np.array([[v_0, last_d, d_sense, a_b, t_react]]), axis=0)
        data_sorted_idx = np.argsort(data[:, 4])
        data = data[data_sorted_idx]
    pd.DataFrame(data, columns=['v_0', 'd(t)', 'd_sense', 'a_b', 't_react']).to_excel(problem_7_path, index=False)
    a_b = list(set(data[:, 3]))
    a_b.sort()
    color = {0:'r', 1:'b', 2:'g', 3:'m', 4:'k'}
    plt.cla()
    for i, j in enumerate(a_b):
        data_partitioned = data[np.where(data[:, 3] == j)[0], :]
        plt.plot(data_partitioned[:, 4], data_partitioned[:, 1], f'{color[i]}o', label=f'd_sense={d_sense}, v_0={v_0}, a_b={j}')
        plt.plot(data_partitioned[:, 4], data_partitioned[:, 1], f'{color[i]}--')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title(f'Comparison of d(t)\nd_sense={d_sense} v_0={v_0}')
    plt.xlabel('t_react')
    plt.ylabel('d(t) = x2(t) - x1(t)')
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.tight_layout()
    plt.savefig(f"{problem_saved_dir}/problem_7.png")
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