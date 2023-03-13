import numpy as np

d = 0.4 # meter.
x = 1.0 # meter.
theta = np.pi - np.arccos((np.sqrt(3)*d)/(x+(x**2 / (2*d*np.sqrt(3)-x))))

print(np.degrees(theta))