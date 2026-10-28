import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


### Waypoints Generation ###

# Lissajous curve parameters
Ax = 1
Ay = 1
omega_x = 3
omega_y = 4
phi_x = np.pi / 2
phi_y = 0

# Parametric equations of the curve
def path(s):
    x = Ax * np.sin(omega_x * s + phi_x)
    y = Ay * np.sin(omega_y * s + phi_y)
    z = 0
    return np.array([x, y, z])

sigma_min, sigma_max = (1/2)*np.pi, (5/2)*np.pi 
num_points = 100 # number of points
sigma = np.linspace(sigma_min, sigma_max, num_points) # scalar parameter array

waypoints = []
for s in sigma:
    waypoints.append(path(s))
waypoints = np.array(waypoints)

# Adapt the waypoints to fit the robot's workspace
scale_factor = 0.15
waypoints = scale_factor * waypoints

# Translate the waypoints w.r.t. the start pose
start_pose = [0.30689056659226,-7.89898763946684e-17,0.480282052303378,-1,0,0,0]
waypoints = waypoints + start_pose[:3] 


### Trajectory Generation ### 

execution_time = 60 # time to execute the trajectory [seconds]
n_int_points = 300 # number of interpolated points
t_values = np.linspace(0, execution_time, num_points) # time horizon

# Cubic (spline) interpolation with null first derivatives as boundary conditions
cs_x = CubicSpline(t_values, waypoints[:, 0], bc_type='clamped')
cs_y = CubicSpline(t_values, waypoints[:, 1], bc_type='clamped')
# since z is constant, I ignore it in the interpolation

# Interpolated points generation
t_int = np.linspace(t_values[0], t_values[-1], n_int_points)
x_int = cs_x(t_int)
y_int = cs_y(t_int)
z_int = waypoints[0, 2] * np.ones(n_int_points) # constant z value

# Add the pre-computed (fixed) orientation
q = start_pose[3:] 
qx_int = q[0] * np.ones(n_int_points)
qy_int = q[1] * np.ones(n_int_points)
qz_int = q[2] * np.ones(n_int_points)
qw_int = q[3] * np.ones(n_int_points)


### Plots ###

# Trajectory plotting (in 2D)
plt.figure(figsize=(10, 6))
plt.plot(x_int, y_int)
plt.scatter(waypoints[:, 0], waypoints[:, 1], color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.grid(True)
plt.show()

# Velocity and acceleration profiles

vx_int = cs_x(t_int, 1)
vy_int = cs_y(t_int, 1)
vel = np.sqrt(vx_int**2 + vy_int**2)

ax_int = cs_x(t_int, 2)
ay_int = cs_y(t_int, 2)
acc = np.sqrt(ax_int**2 + ay_int**2)

fig, axs = plt.subplots(2, 1, figsize=(10, 8))

axs[0].plot(t_int, vel)
axs[0].set_title('Velocity Profile')
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('Velocity [m/s]')
axs[0].grid(True)

axs[1].plot(t_int, acc)
axs[1].set_title('Acceleration Profile')
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Acceleration [m/s^2]')
axs[1].grid(True)

plt.tight_layout()
plt.show()


### Save the trajectory to a CSV file ###

df = pd.DataFrame({'time': t_int, 'x': x_int, 'y': y_int, 'z': z_int, 'qx': qx_int, 'qy': qy_int, 'qz': qz_int, 'qw': qw_int})
df.to_csv('trajectory.csv', index=False)










