import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
df = pd.read_csv("control_values.csv")

# Plot the data
plt.figure(figsize=(10, 6))  # Set the figure size

# Plot throttle, brake, and PID output
plt.plot(df['Throttle'], label='Throttle')
plt.plot(df['Brake'], label='Brake')
plt.plot(df['PID_Output'], label='PID Output')

plt.xlabel('Time Step')
plt.ylabel('Values')
plt.title('Control Values Over Time')
plt.legend()  # Show legend

plt.grid(True)  # Add grid
plt.tight_layout()  # Adjust layout

# Save the plot as an image (optional)
plt.savefig('control_values_plot.png')

# Show the plot
plt.show()
