# Author: Addison Sears-Collins
# Date: September 2, 2021
# Description: Launch a basic mobile robot using the ROS 2 Navigation Stack
# https://automaticaddison.com

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi


def generate_launch_description():

    # Set the path to different files and folders.
    nav2_pkg_path = get_package_share_directory("nav2_bringup")
    gazebo_pkg_path = get_package_share_directory("gazebo_ros")
    pkg_share = get_package_share_directory("basic_mobile_robot")

    world_path = os.path.join(
        pkg_share,
        "worlds",
        "basic_mobile_bot_world",
        "smalltown_with_lidar_robot.world",
    )
    default_model_path = os.path.join(pkg_share, "models", "basic_mobile_bot_v2.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "path_planning.rviz")
    robot_localization_file_path = os.path.join(pkg_share, "config", "ekf.yaml")

    nav2_launch_dir = os.path.join(nav2_pkg_path, "launch")
    nav2_params_path = os.path.join(pkg_share, "params", "nav2_straight_params.yaml")
    
    static_map_path = os.path.join(pkg_share, "maps", "smalltown_world.yaml")
    nav2_bt_path = FindPackageShare(package="nav2_bt_navigator").find(
        "nav2_bt_navigator"
    )
    behavior_tree_xml_path = os.path.join(
        nav2_bt_path, "behavior_trees", "navigate_w_replanning_and_recovery.xml"
    )

    # Launch configuration variables specific to simulation
    autostart = LaunchConfiguration("autostart")
    slam = LaunchConfiguration("slam")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_model_path = os.path.join(pkg_share, "models")
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] += ":" + gazebo_model_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = gazebo_model_path

    print(
        ansi("yellow"),
        "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.",
        ansi("reset"),
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_use_namespace_cmd = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        name="autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name="slam", default_value="False", description="Whether to run SLAM"
    )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Specify the actions

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, "launch", "gzclient.launch.py")
        )
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher = Node(
        # condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro ", default_model_path]),
            }
        ],
        remappings=remappings,
        arguments=[default_model_path],
    )

    # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_file_path, {"use_sim_time": True}],
    )

    # Launch RViz
    rviz2 = Node(
        # condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
    )

    # Launch the ROS 2 Navigation Stack
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_namespace": use_namespace,
            "slam": slam,
            "map": static_map_path,
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_path,
            "default_bt_xml_filename": behavior_tree_xml_path,
            "autostart": autostart,
        }.items(),
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_localization)
    # ld.add_action(nav2_cmd)
    # ld.add_action(rviz2)

    return ld
