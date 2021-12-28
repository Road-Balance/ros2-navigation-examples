from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='cartographer_ros', 
            executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', get_package_share_directory('basic_mobile_robot') + '/config',
                '-configuration_basename', 'cartographer_without_odom.lua'
            ],
            remappings=[
                ('/imu', '/imu/data'),
                ('/odom', '/wheel/odometry'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0'],
        ),
    ]) 