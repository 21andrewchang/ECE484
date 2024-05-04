import numpy as np
import matplotlib.pyplot as plt
import random
import math

def collision_check(x,y,obstacle,obstacle_size):
    for i in range(obstacle.shape[0]):
        distance = np.sqrt((x - obstacle[i][0])**2 + (y-obstacle[i][1])**2)
        if distance < obstacle_size:
            return 0
    else:
        return 1

def vehicle_control(node_x,node_y,node_theta,node_v,node_phy, obstacle, obstacle_size):
    l = 0.5
    theta = node_theta
    x = node_x
    y = node_y
    x_value = []
    y_value = []
    v = node_v
    phi = node_phy # jiaodu
    a = -5 + 10 * random.random()
    psi = -1 + 2 * random.random()
    step = 100 # int(100 * random.random())
    for j in range(step):
        v = v + a * 0.01
        phi = phi + psi * 0.01
        if ((x) > 10 or abs(y) > 10):
            break
        theta = theta + 0.01 * math.tan(phi) * v / l
        x = x + v * math.cos(theta) * 0.01
        y = y + v * math.sin(theta) * 0.01
        value = collision_check(x,y,obstacle,obstacle_size)
        if value:
            x_value.append(x)
            y_value.append(y)
        else:
            return None
    plt.plot(x_value, y_value)
    last_point = [x, y, theta, v, phi]
    return last_point

def generate_random_point(final_node, goal_probability):
    #randomly generate a x_random
    random_x = -10 + 20 * random.random()
    random_y = -10 + 20 * random.random()
    random_theta = 2 * (math.pi) * random.random()
    random_v = -5 + 10 * random.random()
    random_phy = (-1/3)*(math.pi) + (2/3) * (math.pi) * random.random()
    random_node = [random_x,random_y,random_theta,random_v,random_phy]

    #goal_biasing strategy
    goal_biasing = random.random()
    if goal_biasing < goal_probability:
        random_node = final_node
    else:
        random_node = random_node

    return random_node

def find_nearest_node(node_list,w):
    list_distance = []
    for i in range(len(node_list)):
        theta_different = min(abs(random_node[2] - node_list[i][2]),
                              (2 * math.pi - abs(random_node[2] - node_list[i][2])))
        distance = w[0] * (random_node[0] - node_list[i][0]) ** 2 + w[1] * (random_node[1] - node_list[i][1]) ** 2 + w[
            2] * (theta_different) ** 2 + w[3] * (random_node[3] - node_list[i][3]) ** 2 + w[4] * (
                               random_node[4] - node_list[i][4]) ** 2
        list_distance.append(distance)
    tmp = min(list_distance)
    index = list_distance.index(tmp)
    return index

if __name__ == '__main__':
    # initial the node list
    start_node = [0,0,0,0,0]
    node_list = [start_node]
    final_node = [5,5,0,0,0]
    goal_probability = 0.7
    w = [1,1,0,0,0]
    obstacle = np.array([[2,3],[1,5],[3,3]])
    obstacle_size = 0.5

    random_node = generate_random_point(final_node, goal_probability)

    for i in range(1000):
        #find the x_near by brute force or k-d tree
        #by brute force
        index = find_nearest_node(node_list,w)
        #implement the random control and set the new point into tree
        new_point = vehicle_control(node_list[index][0],node_list[index][1],node_list[index][2],node_list[index][3],node_list[index][4],obstacle,obstacle_size)
        if new_point == None:
            continue
        node_list.append(new_point)
        distance = abs(node_list[-1][0] - final_node[0]) + abs(node_list[-1][1] - final_node[1]) # 1d
        if distance < 1:
            break

    print(np.array(node_list).shape)
    plt.scatter(np.array(node_list)[:,0],np.array(node_list)[:,1])
    plt.scatter(obstacle[:,0], obstacle[:,1], marker="o", color = 'red')
    plt.plot(start_node[0],start_node[1],marker = (5,0),markersize = 10,color = 'red')
    plt.plot(final_node[0],final_node[1],marker = (5,0),markersize = 10,color = 'blue')
    plt.title('goal_probability = %f ' % goal_probability)
    plt.show()