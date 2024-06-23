import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sample data for the 3D line plot
x = [0, 1, 2, 3, 4, 5]
y = [0, 1, 4, 9, 16, 25]
z = [0, 1, 2, 3, 4, 5]

# Create the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='3D Line Plot')

# Add a vertical line (pole) at x = 2 and y = 4 spanning from z = 0 to z = 5
ax.plot([2, 2], [4, 4], [0, 5], color='r', linestyle='--', label='Vertical Line')

# Add a horizontal line (pole) at y = 10 and z = 5 spanning from x = 0 to x = 5
ax.plot([0, 5], [10, 10], [5, 5], color='b', linestyle='-.', label='Horizontal Line')

# Add labels and legend
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()

# Show the plot
plt.show()
