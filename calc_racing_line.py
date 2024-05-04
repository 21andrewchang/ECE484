 # Create a directed graph
import csv
import numpy as np
import networkx as nx
import math
import matplotlib.pyplot as plt

# Define empty lists to store data
left_x = []
left_y = []
right_x = []
right_y = []
target_x = []
target_y = []
way_x = []
way_y = []
# Open the CSV file and read data
with open('boundary.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Check if the row has numerical data
        try:
            float(row[0])  # Try converting the first value to float
        except ValueError:
            continue  # Skip this row if conversion fails

        # Assuming each row has 6 columns: Left X, Left Y, Right X, Right Y, Target X, Target Y
        left_x.append(float(row[0]))
        left_y.append(float(row[1]))
        right_x.append(float(row[2]))
        right_y.append(float(row[3]))
        target_x.append(float(row[4]))
        target_y.append(float(row[5]))


heading_angles = []
for i in range(len(target_x) - 1):
    dx = target_x[i + 1] - target_x[i]
    dy = target_y[i + 1] - target_y[i]
    heading_angle = np.arctan2(dy, dx)
    heading_angles.append(heading_angle)

print(len(heading_angles))
learning_rate = 0.1
num_iterations = 500
racing_x, racing_y = np.copy(target_x), np.copy(target_y)

# Optimization loop with heading angle adjustment and gradient descent optimization condition
for _ in range(num_iterations):
    curvature = np.linalg.norm(np.diff(np.column_stack((racing_x, racing_y)), axis=0), axis=1).sum()

    # Calculate heading angle changes between consecutive points
    heading_changes = np.diff(heading_angles)

    # Gradient descent update for x and y coordinates separately based on heading angle condition
    for i in range(len(left_x) - 1):
        if abs(heading_angles[i]) > 0.2 and abs(heading_angles[i]) < 4: #ADJUST THIS VALUE TO SET HEADING ANGLE THRESHOLD
            gradient_x = (racing_x[i + 1] - racing_x[i]) * learning_rate
            gradient_y = (racing_y[i + 1] - racing_y[i]) * learning_rate
            racing_x[i] += gradient_x+0.5*heading_changes[i]
            racing_y[i] += gradient_y+0.5*heading_changes[i]
            # gradient_x = np.gradient(racing_x)
            # gradient_y = np.gradient(racing_y)
            # racing_x[:-2] -= learning_rate * gradient_x[:-2] + 0.5 * learning_rate * heading_changes
            # racing_y[:-2] -= learning_rate * gradient_y[:-2] + 0.5 * learning_rate * heading_changes

    # Apply boundary constraints to keep the racing line within the boundaries
    for i in range(len(racing_x)):
        # Calculate signed distance from the racing line point to the left and right boundaries
        signed_distance_left = np.sqrt((racing_x[i] - left_x[i])**2 + (racing_y[i] - left_y[i])**2)
        signed_distance_right = np.sqrt((racing_x[i] - right_x[i])**2 + (racing_y[i] - right_y[i])**2)

        # Check if the point falls outside the boundaries and adjust if necessary
        # if signed_distance_left > 10**2:
        #     racing_x[i] = left_x[i] + (racing_x[i] - left_x[i]) * 1/3
        #     racing_y[i] = left_y[i] + (racing_y[i] - left_y[i]) * 1/3
        # elif signed_distance_right > 10**2:
        #     racing_x[i] = right_x[i] + (racing_x[i] - right_x[i]) * 1/3
        #     racing_y[i] = right_y[i] + (racing_y[i] - right_y[i]) * 1/3




        #INTELLIGENT X/Y ADJUST
        # if left_x[i] > right_x[i]:
        #   if racing_x[i] > left_x[i] or racing_x[i] < right_x[i]:
        #     racing_x[i] = left_x[i] + (racing_x[i] - left_x[i]) * 1/3
        # else:
        #   if racing_x[i] < left_x[i] or racing_x[i] > right_x[i]:
        #     racing_x[i] = right_x[i] + (racing_x[i] - right_x[i]) * 1/3

        # if left_y[i] > right_y[i]:
        #   if racing_y[i] > left_y[i] or racing_y[i] < right_y[i]:
        #     racing_y[i] = left_y[i] + (racing_y[i] - left_y[i]) * 1/3
        # else:
        #        if racing_y[i] < left_y[i] or racing_y[i] > right_y[i]:
        #             racing_y[i] = right_y[i] + (racing_y[i] - right_y[i]) * 1/3

        factor = 1/3

        if left_x[i] > right_x[i]:
            racing_x[i] = target_x[i] + (left_x[i] - target_x[i]) * factor if racing_x[i] > left_x[i] else target_x[i] + (right_x[i] - target_x[i]) * factor
        else:
            racing_x[i] = target_x[i] + (right_x[i] - target_x[i]) * factor if racing_x[i] > right_x[i] else target_x[i] + (left_x[i] - target_x[i]) * factor

        if left_y[i] > right_y[i]:
            racing_y[i] = target_y[i] + (left_y[i] - target_y[i]) * factor if racing_y[i] > left_y[i] else target_y[i] + (right_y[i] - target_y[i]) * factor
        else:
            racing_y[i] = target_y[i] + (right_y[i] - target_y[i]) * factor if racing_y[i] > right_y[i] else target_y[i] + (left_y[i] - target_y[i]) * factor



print(len(left_x))
print(len(racing_x))


# Plotting
plt.figure(figsize=(8, 6))
plt.plot(left_x, left_y, label='Left Boundary')
plt.plot(right_x, right_y, label='Right Boundary')
plt.plot(target_x, target_y, label='Centerline')
plt.plot(racing_x, racing_y, label='Racing Line', linestyle='--')
plt.legend()
plt.title('Generated Racing Line with Heading Angle Smoothing and Optimization Condition')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.axis('equal')
plt.show()


racing_line_points = list(zip(racing_x, racing_y))

# Define the CSV file path for the racing line
racing_line_csv = 'racing_line.csv'

# Write racing line points to CSV
with open(racing_line_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['X', 'Y'])  # Write header
    writer.writerows(racing_line_points)

print(f'Racing line points exported to {racing_line_csv}')

# Plotting
# fig, ax = plt.subplots(figsize=(8, 6))
# ax.plot(left_x, left_y, label='Left Boundary')
# ax.plot(right_x, right_y, label='Right Boundary')
# ax.plot(target_x, target_y, label='Centerline')
# racing_line, = ax.plot(racing_x, racing_y, label='Racing Line', linestyle='--')
# ax.legend()
# ax.set_title('Generated Racing Line with Heading Angle Smoothing and Optimization Condition')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.grid(True)
# ax.axis('equal')

# # Function to calculate heading angle given (x, y) coordinates
# def calculate_heading_angle(x, y):
#     index = np.argmin(np.abs(np.array(target_x) - x))
#     dx = target_x[index + 1] - target_x[index]
#     dy = target_y[index + 1] - target_y[index]
#     heading_angle = np.arctan2(dy, dx)
#     return heading_angle

# # Event handler for mouse click
# def onclick(event):
#     if event.inaxes == ax:
#         x_clicked, y_clicked = event.xdata, event.ydata
#         heading_angle_clicked = calculate_heading_angle(x_clicked, y_clicked)
#         print(f'Clicked at (X={x_clicked:.2f}, Y={y_clicked:.2f}) - Heading Angle: {heading_angle_clicked:.2f} rad')

# # Connect the event handler to the figure
# fig.canvas.mpl_connect('button_press_event', onclick)

# plt.show()