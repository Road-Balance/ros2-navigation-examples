import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi

import xacro

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('basic_mobile_robot'))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    world_path = os.path.join(pkg_path, 'worlds', 'basic_mobile_bot_world', 'smalltown_with_lidar_robot.world')
    robot_localization_file_path = os.path.join(pkg_path, 'config','ekf.yaml') 
    
    gazebo_model_path = os.path.join(pkg_path, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))


    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path, 
            {'use_sim_time': True}
        ]
    )

    # Robot State Publisher
    urdf_file = os.path.join(pkg_path, 'models', 'basic_mobile_bot_v2.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    param = {'use_sim_time': False, 'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[param]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'basic_mobile_robot'],
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'robot_localization.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        joint_state_publisher,
        robot_localization_cmd,
        rqt_robot_steering,
        # TimerAction(
        #     period=3.0,
        #     actions=[rviz]
        # ),
    ])