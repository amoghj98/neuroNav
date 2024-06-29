import numpy as np
import matplotlib.pyplot as plt

def read_and_plot_trajectory(folder_path, label):
    # Read position data
    x = np.load(f"{folder_path}/xPos.npy")
    y = np.load(f"{folder_path}/yPos.npy")
    z = np.load(f"{folder_path}/zPos.npy")

    # Create 3D plot for the trajectories with thicker lines
    ax.plot(x, y, z, label=label, linewidth=2.5)  # Increased line width for better visibility

# Create a 3D plot with a larger figure size
fig = plt.figure(figsize=(8, 10))
ax = fig.add_subplot(111, projection='3d')

# Plotting the EV-Planner and PID trajectories
read_and_plot_trajectory('/home/sourav/icra40/EV-Planner', 'EV-Planner')
read_and_plot_trajectory('/home/sourav/icra40/PID', 'EV-PID')

# Adding labels and title with increased font sizes and adjusted padding
# ax.set_xlabel('X Position', fontsize=16, labelpad=15)
# ax.set_ylabel('Y Position', fontsize=16, labelpad=15)
# ax.set_zlabel('Z Position', fontsize=16, labelpad=15)
# ax.set_title('Trajectories of Drone using EV-Planner and EV-PID', fontsize=18)

# Set larger font sizes for the axis tick labels
ax.tick_params(axis='both', which='major', labelsize=12)


# Show the plot
plt.show()
