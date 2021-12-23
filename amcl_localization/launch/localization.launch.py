import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('amcl_localization'), 'config', 'nav2.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('amcl_localization'), 'rviz', 'nav2_default_view.rviz')
    map_file = os.path.join(get_package_share_directory('amcl_localization'), 'map', 'turtlebot3_world.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_yaml, 
            {'yaml_filename':map_file}
        ]
    )

    map_service_call_param = "{map_url: " + map_file + "}"
    map_service_call = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/map_server/load_map', 
            'nav2_msgs/srv/LoadMap', map_service_call_param],
        output='log'
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        node_executable='amcl',
        node_name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[map_service_call]
        ),
        rviz2,
        lifecycle_manager,
        map_server,
        nav2_amcl,
    ])