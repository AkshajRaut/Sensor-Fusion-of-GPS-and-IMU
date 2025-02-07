import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

file_path = 'circles_imu.csv'
data = pd.read_csv(file_path)

data['seconds'] = data['seconds'] + data['nanoseconds'] / 1e9

data = data[3485:6009]

magnetic_x = data['Magnetic.x'].to_numpy()
magnetic_y = data['Magnetic.y'].to_numpy()

x_offset = (magnetic_x.min() + magnetic_x.max()) / 2.0
y_offset = (magnetic_y.min() + magnetic_y.max()) / 2.0

calibrated_x = magnetic_x - x_offset
calibrated_y = magnetic_y - y_offset

radius = np.sqrt(calibrated_x[1500]**2 + calibrated_y[1500]**2)
theta = np.arcsin(calibrated_y[1500] / radius)

R = np.array([[np.cos(theta), np.sin(theta)], 
              [-np.sin(theta), np.cos(theta)]])

soft_iron_calibrated = np.dot(R, np.array([calibrated_x, calibrated_y]))

soft_iron_calibrated_x, soft_iron_calibrated_y = soft_iron_calibrated[0], soft_iron_calibrated[1]

plt.figure(figsize=(6, 6))
plt.scatter(magnetic_x, magnetic_y, label='Uncalibrated Data', color='b', s=2, alpha=0.5)
plt.title('Uncalibrated Magnetic Data')
plt.xlabel('Magnetic X (Tesla)')
plt.ylabel('Magnetic Y (Tesla)')
plt.xlim([-0.2, 0.2])
plt.ylim([-0.2, 0.2])
plt.legend()
plt.grid()
plt.axis('equal')
plt.gca().add_patch(plt.Circle((0, 0), 0.000023, color='black', fill=False, linestyle='-', linewidth=1.5, zorder=10))
plt.show()

plt.figure(figsize=(6, 6))
plt.scatter(calibrated_x, calibrated_y, label='Hard-Iron Calibrated Data', color='crimson', s=2, marker='+')
plt.title('Hard-Iron Calibrated Magnetic Data')
plt.xlabel('Magnetic X (Tesla)')
plt.ylabel('Magnetic Y (Tesla)')
plt.xlim([-0.2, 0.2])
plt.ylim([-0.2, 0.2])
plt.legend()
plt.grid()
plt.axis('equal')
plt.gca().add_patch(plt.Circle((0, 0), 0.000023, color='black', fill=False, linestyle='-', linewidth=1.5, zorder=10))
plt.show()

plt.figure(figsize=(6, 6))
plt.scatter(soft_iron_calibrated_x, soft_iron_calibrated_y, label='Soft-Iron Calibrated Data', color='green', s=2, marker='x')
plt.title('Soft-Iron Calibrated Magnetic Data')
plt.xlabel('Magnetic X (Tesla)')
plt.ylabel('Magnetic Y (Tesla)')
plt.xlim([-0.2, 0.2])
plt.ylim([-0.2, 0.2])
plt.legend()
plt.grid()
plt.axis('equal')
plt.gca().add_patch(plt.Circle((0, 0), 0.000023, color='black', fill=False, linestyle='-', linewidth=1.5, zorder=10))
plt.show()
