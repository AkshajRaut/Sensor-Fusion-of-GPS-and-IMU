import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from msg_package.msg import IMUmsg
from datetime import datetime,timezone
from builtin_interfaces.msg import Time
import numpy as np
import serial
import time
import math

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('IMU_Data_Pub')
        SENSOR_NAME ="VN-100"
        self.declare_parameter('imu_port', '')
        self.declare_parameter('imu_baudrate', 115200)
        self.declare_parameter('imu_frequency', 40)
        self.publisher_ = self.create_publisher(IMUmsg, '/imu', 5)
        self.get_logger().debug("Initialization complete")
        self.get_logger().info("Publishing IMU Data")

        ser_port = self.get_parameter('imu_port').value
        ser_baud = int(self.get_parameter('imu_baudrate').value)
        frequency = self.get_parameter('imu_frequency').value
        self.port = serial.Serial(ser_port, ser_baud, timeout=3.0)
        self.get_logger().debug("Starting " + SENSOR_NAME + "on the serial port" + ser_port + " at baud rate " + str(ser_baud))
        
        self.sleep_time = 1 / frequency
        self.timer = self.create_timer(self.sleep_time, self.imu_data_publish)
        self.get_logger().info ("GPS Driver Launched")
        freq_string = "$VNWRG,07,"+ str(frequency)

        checksum = 0
        for char in freq_string[1:]:
            checksum ^= ord(char) 
        
        hexa_checksum = f"{checksum:02X}"

        freq_send = f'$VNWRG,07,{frequency}*{hexa_checksum}\r\n'
        self.port.write (freq_send.encode()) 


    def imu_data_publish(self):

        inc_data = self.port.readline().decode('ascii', errors='replace').strip()

        if inc_data.startswith('$VNYMR'):
            or_str = IMUmsg()
            new_str = inc_data.split(',')

            if len(new_str)!=13 or new_str[0] != '$VNYMR':
                return None
            
            try:
            
                or_str.header = Header()
                or_str.header.frame_id = 'IMU1_Frame'

                current_datetime = datetime.now()
                seconds = current_datetime.hour*3600 + current_datetime.minute*60 + current_datetime.second
                nseconds = seconds + current_datetime.microsecond*1000
                or_str.header.stamp.sec = seconds
                or_str.header.stamp.nanosec = nseconds

                yaw = float(new_str[1])
                
                pitch = float(new_str[2])
                
                roll = float(new_str[3])
                

                or_str.mag_field.magnetic_field.x = float(new_str[4])/10000
                or_str.mag_field.magnetic_field.y = float(new_str[5])/10000
                or_str.mag_field.magnetic_field.z = float(new_str[6])/10000
                or_str.imu.linear_acceleration.x = float(new_str[7])
                or_str.imu.linear_acceleration.y = float(new_str[8])
                or_str.imu.linear_acceleration.z = float(new_str[9])
                or_str.imu.angular_velocity.x = float(new_str[10])
                or_str.imu.angular_velocity.y = float(new_str[11])
                or_str.imu.angular_velocity.z = float(new_str[12].split('*')[0])
                or_str.yaw = yaw
                or_str.pitch = pitch
                or_str.roll = roll

                or_str.imu.orientation.x,or_str.imu.orientation.y,or_str.imu.orientation.z,or_str.imu.orientation.w = euler_to_quarternion(yaw,pitch,roll)

                self.publisher_.publish(or_str)
               

            except ValueError:
                return None
        
# Convert IMU values to quarternion

def euler_to_quarternion(yaw,pitch,roll):

    R = roll * (math.pi/180)
    P = pitch * (math.pi/180)
    Y = yaw * (math.pi/180)

    qx = (np.sin(R/2) * np.cos(P/2) * np.cos(Y/2)) - (np.cos(R/2) * np.sin(P/2) * np.sin(Y/2))
    qy = (np.cos(R/2) * np.sin(P/2) * np.cos(Y/2)) + (np.sin(R/2) * np.cos(P/2) * np.sin(Y/2))
    qz = (np.cos(R/2) * np.cos(P/2) * np.sin(Y/2)) - (np.sin(R/2) * np.sin(P/2) * np.cos(Y/2))
    qw = (np.cos(R/2) * np.cos(P/2) * np.cos(Y/2)) + (np.sin(R/2) * np.sin(P/2) * np.sin(Y/2))
    return qx,qy,qz,qw

def main(args=None):

    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.port.close()
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
