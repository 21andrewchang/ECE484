import matplotlib.pyplot as plt
import numpy as np

# Define start and goal nodes
start_node = (10, 10)
goal_node = (90, 90)

# Parameters for realistic tree generation
num_trees = 5
tree_length = 50
max_turn_angle = np.radians(30)  # Maximum steering angle for the vehicle
goal_bias_prob = 0.5  # Probability of biasing towards the goal
distance_threshold = 30  # Distance threshold to directly steer towards the goal

# Helper function to bias towards the goal
def get_random_angle(goal_pos, curr_pos, dist_to_goal):
    if np.random.uniform(0, 1) < goal_bias_prob and dist_to_goal > distance_threshold:
        # Calculate angle towards the goal
        angle_to_goal = np.arctan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0])
        # Generate random angle biased towards the goal angle
        return np.random.uniform(angle_to_goal - max_turn_angle, angle_to_goal + max_turn_angle)
    else:
        # Generate a random angle within the maximum turn angle
        return np.random.uniform(-max_turn_angle, max_turn_angle)

# Generate random decision trees with goal bias
tree_nodes = []
for _ in range(num_trees):
    x_vals = [start_node[0]]
    y_vals = [start_node[1]]
    curr_pos = np.array(start_node)
    for _ in range(tree_length):
        dist_to_goal = np.linalg.norm(curr_pos - np.array(goal_node))
        steering_angle = get_random_angle(goal_node, curr_pos, dist_to_goal)
        move_distance = np.random.uniform(1, 5)  # Random distance moved
        new_pos = curr_pos + move_distance * np.array([np.cos(steering_angle), np.sin(steering_angle)])
        new_pos = np.clip(new_pos, 0, 100)  # Ensure within bounds
        x_vals.append(new_pos[0])
        y_vals.append(new_pos[1])
        curr_pos = new_pos
    tree_nodes.append(list(zip(x_vals, y_vals)))

# Plotting
plt.figure(figsize=(8, 8))
plt.scatter(start_node[0], start_node[1], color='green', label='Start Node', s=100, marker='o')
plt.scatter(goal_node[0], goal_node[1], color='red', label='Goal Node', s=100, marker='x')

for nodes in tree_nodes:
    x_vals, y_vals = zip(*nodes)
    plt.plot(x_vals, y_vals, linestyle='-', marker='.', markersize=5)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Realistic Vehicle Exploration Tree with Goal Bias (RRT Demo)')
plt.legend()
plt.grid(True)
plt.xlim(0, 100)
plt.ylim(0, 100)
plt.show()
