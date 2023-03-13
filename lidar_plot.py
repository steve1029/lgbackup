import numpy as np
import matplotlib.pyplot as plt

vx = 60*1000/3600
vy = 60*1000/3600
x0 = -45 * np.sqrt(3)
y0 = -45
l0 = np.sqrt(x0**2 + y0**2)
theta = np.arctan(x0/y0)

FPS = 30 
dt = 1/FPS
dx = vx*dt
dy = vy*dt
x1 = x0 + dx
y1 = y0 + dy
l1 = np.sqrt(x1**2+y1**2)

v = np.sqrt(vx**2+vy**2)
g = vx*x0+vy*y0
dl = l1 - l0

npixels_x = 2*x0 / dx
npixels_y = 2*y0*np.tan(np.pi/12) / dy

al_stop_f = 10 # Required number of frames for algorithm determine to stop of not.

stop_t = al_stop_f / FPS

vy = np.linspace(0,100,101)
d_lidar = (vy*stop_t + abs(y0)) / np.cos(theta)
dot_density = 6.34 # unit = 1/m^2.
angular_resolution = np.arctan(1/d_lidar/np.sqrt(dot_density)) * 180 / np.pi

d_lidar = lambda v,fps: (v*1000/3600/fps*10 + abs(y0)) / np.cos(theta)

fig, axis = plt.subplots(nrows=1, ncols=1, figsize=(5,5))

for fps in [10, 20, 25, 30]:
    dd = d_lidar(60, fps)
    p, = axis.plot(vy, d_lidar(vy,fps), label=f"{fps} FPS")
    axis.plot(60, dd, marker='o', c=p.get_color(), zorder=1)
    angular_resolution = np.arctan(1/dd/np.sqrt(dot_density)) * 180 / np.pi
    print(dd, angular_resolution, vx*10/fps)

axis.axvline(60, color='grey', lw=0.4, zorder=0)
axis.grid(True, axis='y', lw=0.4)
axis.set_xlabel("Velocity (km/h)", fontsize=15)
axis.set_ylabel("Distance (m)", fontsize=15)
axis.legend(loc='upper left', fontsize=15)
axis.tick_params(axis='both', which='major', labelsize=15)
fig.savefig("test.png", bbox_inches='tight')