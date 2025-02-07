import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

imu_data = pd.read_csv('trip_imu.csv')
gps_data = pd.read_csv('trip_gps.csv')

imu_data['time'] = imu_data['seconds'] + imu_data['nanoseconds'] / 1e9
gps_data['time'] = gps_data['seconds'] + gps_data['nanoseconds'] / 1e9

imu_data['time'] -= imu_data['time'].iloc[0]
gps_data['time'] -= gps_data['time'].iloc[0]

linear_acc_x = imu_data['Linear.x'].to_numpy()
imu_time = imu_data['time'].to_numpy()

linear_acc_x -= np.mean(linear_acc_x)

forward_velocity_imu = cumtrapz(linear_acc_x, imu_time, initial=0)

utm_easting = gps_data['UTM Easting'].to_numpy()
utm_northing = gps_data['UTM Northing'].to_numpy()
gps_time = gps_data['time'].to_numpy()

distance = np.sqrt(np.diff(utm_easting)**2 + np.diff(utm_northing)**2)
gps_velocity = distance / np.diff(gps_time)

gps_time = gps_time[1:]

plt.figure(figsize=(12, 6))
plt.plot(imu_time, forward_velocity_imu, label='IMU-based Velocity', color='blue')
plt.plot(gps_time, gps_velocity, label='GPS-based Velocity', color='orange')
plt.title('Forward Velocity from IMU and GPS')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()

adjusted_velocity_imu = np.copy(forward_velocity_imu)
adjusted_velocity_imu[adjusted_velocity_imu < 0] = 0

plt.figure(figsize=(12, 6))
plt.plot(imu_time, adjusted_velocity_imu, label='Corrected IMU-based Velocity', color='green')
plt.plot(gps_time, gps_velocity, label='GPS-based Velocity', color='orange')
plt.title('Corrected Forward Velocity from IMU and GPS')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()
