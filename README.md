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

가제보에 bocbot을 띄우기만 해도 topic이 나오는가?
bocbot world에 basic mobile bot을 띄워놓고 저장, 이걸로 launch 파일을 수정한다.

```