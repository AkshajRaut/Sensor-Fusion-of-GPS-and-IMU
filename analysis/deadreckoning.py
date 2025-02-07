
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumtrapz

imu_data = pd.read_csv('trip_imu.csv')
gps_data = pd.read_csv('trip_gps.csv')

imu_data['time'] = imu_data['seconds'] + imu_data['nanoseconds'] / 1e9
gps_data['time'] = gps_data['seconds'] + gps_data['nanoseconds'] / 1e9

imu_time = imu_data['time'].to_numpy()
gps_time = gps_data['time'].to_numpy()

gyro_z = imu_data['Angular.z'].to_numpy()
accel_x = imu_data['Linear.x'].to_numpy()
accel_y = imu_data['Linear.y'].to_numpy()
gps_easting = gps_data['UTM Easting'].to_numpy()
gps_northing = gps_data['UTM Northing'].to_numpy()
yaw_i = imu_data['Yaw'].to_numpy()


imu_data['time'] -= imu_data['time'].iloc[0]
gps_data['time'] -= gps_data['time'].iloc[0]


yaw_imu = np.unwrap(yaw_i)


ac_c = accel_x - np.mean(accel_x)
vel_ac_c = cumtrapz(ac_c, imu_time, initial=0)
vel_ac_c[vel_ac_c < 0] = 0

disp = cumtrapz(vel_ac_c, initial=0)
x2dot = accel_x
x1dot = cumtrapz(accel_x, imu_time, initial=0)
y2dot = gyro_z * x1dot
Y_obs = accel_y
fv = vel_ac_c
ve = fv * np.sin(np.radians(yaw_imu - 19))
vn = fv * np.cos(np.radians(yaw_imu - 19))
xe = cumtrapz(ve, imu_time, initial=0)
xn = cumtrapz(vn, imu_time, initial=0)

xe_s = xe
xn_s = xn
xe_a = xe_s - xe_s[0] + gps_easting[0]
xn_a = xn_s - xn_s[0] + gps_northing[0]

plt.figure(figsize=(10, 6))
plt.plot(gps_easting, gps_northing, label='IMU Trajectory', color='green')
plt.plot(xe_a, xn_a,label='GPS Trajectory',  color='blue')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Comparison of Estimated and GPS Trajectory')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 10))
plt.plot(Y_obs, label='Y observed', color='blue')
plt.plot(y2dot / -1, label='wX(dot)', color='green')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Y Observed vs wX(dot)')
plt.xlabel('Samples @ 40Hz')
plt.ylabel('Acceleration (m/s²)')
plt.show()
