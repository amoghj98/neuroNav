import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
 
# Points
x = np.array([-0.7, 1])
y = np.array([1.25, 0])
 
# Fit the cubic spline
cs = CubicSpline(x, y)
 
# Generate fine points for plotting
x_fine = np.linspace(-0.7, 1, 100)
y_fine = cs(x_fine)

print(cs(0.3))
 
# Plotting
# plt.figure(figsize=(8, 4))
# plt.plot(x, y, 'o', label='Data points')
# plt.plot(x_fine, y_fine, label='Cubic spline')
# plt.title('Cubic Spline Interpolation')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.legend()
# plt.grid(True)
# plt.show()