
import launch.actions
import launch.substitutions

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mapviz = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
    )

    odom_static_transform_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["100", "200", "5", "0.3", "0", "0", "map", "odom"]
    )

    map_static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )


    return LaunchDescription([
        mapviz,
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"name": "local_xy_frame", "value": "map"},
                {"name": "local_xy_origin", "value": "swri"},
                {"name": "local_xy_origins", "value": """[
                    {"name": "swri",
                        "latitude": 47.8405,
                        "longitude": 10.6199,
                        "altitude": 741.48,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": 29.447507,
                        "longitude": -98.629367,
                        "altitude": 200.0,
                        "heading": 0.0}
                ]"""}
            ]
        ),
        odom_static_transform_publisher,
        map_static_transform_publisher,
    ])