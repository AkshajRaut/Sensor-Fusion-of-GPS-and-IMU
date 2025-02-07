# Sensor-Fusion-of-GPS-and-IMU
Built a navigation stack using two different sensors - GPS &amp; IMU, understand their relative strengths + drawbacks, and get an introduction to sensor fusion.


# This is LAB4 of EECE5554 - Robotics Sensing and Navigation


### Our group conducted two experiments for this lab :


a. Magnetometer Calibration to remove hard and soft iron errors


b. Collection of Magnetometer, Gyroscope data in a driving car, to design a complementary filter to match with the yaw estimation of the VN-100 IMU


c. Collection of GPS and Accelerometer Data to calculate Forward velocity of a moving car


d. Using the previously estimated filtered yaw and forward velocity to map the trajectory of the moving car.



### The following are the steps we took to write our ROS2 driver


a. We placed the GPS Driver and IMU Driver in /src of our workspace


b. We wrote a custom launch file in imu_driver/launch called sensor_launch.py


### The following are the steps to run our ROS2 driver we wrote to record data from VN-100 IMU and BU-353 GPS Modules


a. cd to the folder


b. Run the following command


ros2 launch imu_driver sensor_launch.py imu_port:=/dev/ttyUSB0 gps_port:=/dev/ttyACM0 imu_baudrate:="115200" imu_frequency:=40


c. Then, in two separate terminals, we echoed the /gps and /imu topics to check if the data is coming continuously


ros2 topic echo /gps


ros2 topic echo /imu





