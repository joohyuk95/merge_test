import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

# Simulated noisy data points
np.random.seed(42)
num_points = 20
true_slope = -1.5  # Negative slope
true_intercept = 10.0
noise = np.random.normal(0, 2, num_points)
x = np.linspace(0, 10, num_points)
y = true_slope * x + true_intercept + noise

# Reshape the data for sklearn
x_reshaped = x.reshape(-1, 1)

# Fit a linear regression model
model = LinearRegression()
model.fit(x_reshaped, y)

# Get the estimated slope and intercept
estimated_slope = model.coef_[0]
estimated_intercept = model.intercept_

print(f"True Slope: {true_slope:.2f}, True Intercept: {true_intercept:.2f}")
angle_rad = np.arctan(estimated_slope)
angle_deg = np.degrees(angle_rad)

print("Estimated Slope:", estimated_slope)
print("Angle (Radians):", angle_rad)
print("Angle (Degrees):", angle_deg)

# Plot the original noisy data points and the estimated line
plt.scatter(x, y, label="Noisy Data")
plt.plot(x, estimated_slope * x + estimated_intercept, color='red', label="Estimated Line")
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.show()