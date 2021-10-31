# ros2-navigation-examples


```
cbp bocbot
cbp basic_mobile_robot
roseloq


export LC_NUMERIC=en_US.UTF-8

ros2 launch bocbot bocbot_viz.launch.py
ros2 launch basic_mobile_robot mobile_robot_viz.launch.py

ros2 launch bocbot world.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard __ns:=/bocbot

ros2 launch basic_mobile_robot world.launch.py
ros2 launch basic_mobile_robot basic_mobile_bot_v2.launch.py

ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py
rqt_robot_steering

```