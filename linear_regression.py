import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

# Simulated noisy data points
np.random.seed(42)
num_points = 20
true_slope = 2.0
true_intercept = 5.0
noise = np.random.normal(0, 2, num_points)
x = np.linspace(0, 10, num_points)
y = true_slope * x + true_intercept + noise
print("type")
print(type(x), np.shape(x))
print(type(y), np.shape(y))
# Reshape the data for sklearn
x_reshaped = x.reshape(-1, 1)
print(np.shape(x_reshaped), np.shape(y))
# Fit a linear regression model

x = np.array([ 0.00000000e+00, -3.33444475e-02, -4.46235470e-05, -6.68357086e-02,
                    3.31475931e-02, -1.00233079e-01,  6.60838141e-02, -1.34364312e-01,
                    9.90973271e-02, -1.68120097e-01,  1.31786312e-01, -2.01914959e-01,
                    1.64724847e-01, -2.35896844e-01,  1.97065807e-01, -2.70499654e-01,
                    2.29354038e-01, -3.04001870e-01,  2.61888702e-01, -3.39336462e-01,
                    2.93724825e-01])

y = np.array([9.51249027, 9.74712767, 9.69643707, 9.76802867, 9.8035509 ,
                   9.96587923, 10.46675791, 10.61715587, 11.46173082, 11.62623473,
                   10.43580917, 10.6339555 , 9.43436833, 9.64691653, 9.40469769,
                   9.67864722, 9.38115461, 9.66811141, 9.97211893, 9.71191471,
                   9.34267015])


x_reshaped = x.reshape(-1, 1)
model = LinearRegression()
model.fit(x_reshaped, y)

# Get the estimated slope and intercept
estimated_slope = model.coef_[0]
estimated_intercept = model.intercept_

direction_vector = np.array([1, estimated_slope])

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
# plt.gca().set_aspect('equal', adjustable='box')
print(x_reshaped, y)
plt.show()