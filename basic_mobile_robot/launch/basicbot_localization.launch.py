import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('basic_mobile_robot'), 'params', 'localization_params.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('basic_mobile_robot'), 'rviz', 'nav2_localization.rviz')
    map_file = os.path.join(get_package_share_directory('basic_mobile_robot'), 'maps', 'smalltown_world.yaml')

    map_server = Node(
        package='nav2_map_server',
        node_executable='map_server',
        node_name='map_server',
        output='screen',
        parameters=[
            nav2_yaml,
            {'yaml_filename':map_file} 
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        node_executable='amcl',
        node_name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager,
        rviz2
    ])