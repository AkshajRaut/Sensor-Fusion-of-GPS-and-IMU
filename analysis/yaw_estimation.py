import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from scipy.signal import butter, filtfilt

file_path = 'trip_imu.csv'
data = pd.read_csv(file_path)

data['time'] = data['seconds'] + data['nanoseconds'] / 1e9
time = data['time'].to_numpy()

data['time'] -= data['time'].iloc[0]


magnetic_x = data['Magnetic.x'].to_numpy()
magnetic_y = data['Magnetic.y'].to_numpy()

x_offset = (magnetic_x.min() + magnetic_x.max()) / 2.0
y_offset = (magnetic_y.min() + magnetic_y.max()) / 2.0

calibrated_x = magnetic_x - x_offset
calibrated_y = magnetic_y - y_offset

radius = np.sqrt(calibrated_x[2000]**2 + calibrated_y[2000]**2)
theta = np.arcsin(calibrated_y[2000] / radius)

R = np.array([[np.cos(theta), np.sin(theta)], 
              [-np.sin(theta), np.cos(theta)]])

soft_iron_calibrated = np.dot(R, np.array([calibrated_x, calibrated_y]))

soft_iron_calibrated_x, soft_iron_calibrated_y = soft_iron_calibrated[0], soft_iron_calibrated[1]

raw_yaw = np.arctan2(magnetic_y, magnetic_x) * (180 / np.pi)
corrected_yaw = np.arctan2(soft_iron_calibrated_y, soft_iron_calibrated_x) * (180 / np.pi)

raw_yaw_unwrapped = np.unwrap(np.radians(raw_yaw)) * (180 / np.pi)
corrected_yaw_unwrapped = np.unwrap(np.radians(corrected_yaw)) * (180 / np.pi)

imu_yaw = data['Yaw'].to_numpy()
imu_yaw_unwrapped = np.unwrap(imu_yaw)

yaw_rate = data['Angular.z'].to_numpy()
yaw_angle_from_gyro_deg = cumtrapz(yaw_rate, time, initial=0) * (180 / np.pi)

def apply_lpf(data, cutoff, fs, order=3):
    b, a = butter(order, cutoff / (fs / 2), 'low')
    return filtfilt(b, a, data)

def apply_hpf(data, cutoff, fs, order=3):
    b, a = butter(order, cutoff / (fs / 2), 'high')
    return filtfilt(b, a, data)

fs = 40
cutoff_lpf = 0.1
cutoff_hpf = 0.0001
lpf_yaw_mag = apply_lpf(corrected_yaw_unwrapped, cutoff_lpf, fs)
hpf_yaw_gyro = apply_hpf(yaw_angle_from_gyro_deg, cutoff_hpf, fs)

alpha = 0.88
complementary_yaw = alpha * lpf_yaw_mag + (1 - alpha) * hpf_yaw_gyro

plt.figure(figsize=(14, 6))
plt.plot(time, lpf_yaw_mag, label='Low-pass Filtered Magnetometer Yaw', color='blue', alpha=0.6)
plt.plot(time, hpf_yaw_gyro, label='High-pass Filtered Gyro Yaw', color='green', alpha=0.6)
plt.plot(time, complementary_yaw, label='Complementary Filter Yaw', color='purple', alpha=0.8)
plt.title('Comparison of LPF, HPF, and Complementary Filter Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

plt.figure(figsize=(14, 6))
plt.plot(time, imu_yaw_unwrapped, label='Raw Yaw from IMU', color='orange', alpha=0.6)
plt.plot(time, complementary_yaw, label='Complementary Filter Yaw', color='purple', alpha=0.8)
plt.title('Comparison of Raw IMU Yaw and Complementary Filter Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()


plt.figure(figsize=(14, 6))
plt.plot(time, corrected_yaw_unwrapped, label='Corrected Magnetometer Yaw', color='orange', alpha=0.6)
plt.plot(time, yaw_angle_from_gyro_deg, label='Yaw Integrated from Gyro', color='purple', alpha=0.8)
plt.title('Comparison of Magnetometer Yaw and Yaw Integrated from Gyro')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
