# ros2-navigation-examples


```
cbp bocbot
cbp basic_mobile_robot
roseloq


export LC_NUMERIC=en_US.UTF-8

# V1 robot description
ros2 launch bocbot bocbot_viz.launch.py
ros2 launch basic_mobile_robot mobile_robot_viz.launch.py
ros2 launch basic_mobile_robot basic_mobile_bot_v1.launch.py

ros2 launch bocbot world.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard __ns:=/bocbot

# V2 robot in gazebo
ros2 launch basic_mobile_robot world.launch.py
ros2 launch basic_mobile_robot basic_mobile_bot_v2.launch.py

# V4 Lidar added
ros2 launch basic_mobile_robot basic_mobile_bot_v4.launch.py

# V5 basic navigation
ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py

```

## Utils

```
ros2 run tf2_tools view_frames.py

```