import matplotlib.pyplot as plt

# Define the limits of x and y
x_min, x_max = -2, 2
y_min, y_max = -2, 2

start = (-1, -2)
goal = (2, 2)

# Create points and plot them with connecting lines
for x in range(x_min, x_max + 1):
    for y in range(y_min, y_max + 1):
        plt.scatter(x, y, color="black", zorder=2)  # Plot points

        if x < x_max:
            plt.plot([x, x + 1], [y, y], color="black", zorder=1)  # Plot horizontal lines

        if y < y_max:
            plt.plot([x, x], [y, y + 1], color="black", zorder=1)  # Plot vertical lines

        # Plot diagonal lines
        if x < x_max and y < y_max:
            plt.plot([x, x + 1], [y, y + 1], color="black", zorder=1)  # Diagonal up-right
        
        if x < x_max and y > y_min:
            plt.plot([x, x + 1], [y, y - 1], color="magenta", zorder=1)  # Diagonal down-right

plt.scatter(start[0], start[1], color=(1,0,0), zorder=2)
plt.scatter(goal[0], goal[1], color=(0,1,0), zorder=2)

# Set labels, title, and range of axes
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Rectangular Grid with Points and Lines (including Diagonals)")

plt.axis([x_min - 0.5, x_max + 0.5, y_min - 0.5, y_max + 0.5])
plt.axis('equal') 
plt.grid(True)

# Display the plot
plt.show()
