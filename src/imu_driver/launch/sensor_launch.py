from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_port',
            default_value = '/dev/ttyUSB0',
            description = 'Serial'
        ),
        
        DeclareLaunchArgument(
            'gps_port',
            default_value = '/dev/ttyACM0',
            description = 'Serial'
        ),

        DeclareLaunchArgument(
            'imu_baudrate',
            default_value = '115200',
            description = 'Baudrate'
        ),

        DeclareLaunchArgument(
            'imu_frequency',
            default_value = '40',
            description = 'Frequency of IMU Readings'
        ),

        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu_driver_data',
            output='screen',
            parameters = [{'imu_port' : LaunchConfiguration('imu_port'), 
                           'imu_baudrate' : LaunchConfiguration('imu_baudrate'), 
                           'imu_frequency' : LaunchConfiguration('imu_frequency')
            }]
        ),

        Node(
            package='gps_driver',
            executable='gps_driver',
            name='gps_driver_data',
            output='screen',
            parameters = [{'gps_port' : LaunchConfiguration('gps_port')}]
        )
    ])
