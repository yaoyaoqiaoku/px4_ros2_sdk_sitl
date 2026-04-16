#!/bin/bash

# 启动PX4 SITL仿真
gnome-terminal --window --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam; exec bash"

# 等待PX4启动
sleep 5

# 启动MicroXRCEAgent
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "cd ~/px4_ros2_sdk_sitl && source install/setup.bash && MicroXRCEAgent udp4 -p 8888; exec bash"

# 启动ROS2节点
gnome-terminal --tab --title="PX4 Estimator" -- bash -c "sleep 3; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_control px4_estimator.launch.py; exec bash"

# 启动PX4控制节点
gnome-terminal --tab --title="PX4 Control" -- bash -c "sleep 4; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_control px4_control.launch.py; exec bash"

# 启动ArucoLand节点
gnome-terminal --tab --title="ArucoLand" -- bash -c "sleep 5; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch precision_land precision_landing_system.launch.py; exec bash"

# 启动舵机控制节点
gnome-terminal --tab --title="Servo Controller" -- bash -c "sleep 6; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_control px4_servo_controller.launch.py; exec bash"

# 启动PX4 MISSION任务节点
gnome-terminal --tab --title="Mission" -- bash -c "sleep 7; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_control px4_mission.launch.py; exec bash"

# 启动PX4 Home节点
gnome-terminal --tab --title="Home Position" -- bash -c "sleep 6; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_control px4_home_controller.launch.py; exec bash"

# MQTT节点

# 启动MQTT PX4控制节点
gnome-terminal --tab --title="MQTT Control" -- bash -c "sleep 8; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_mqtt mqtt_control.launch.py; exec bash"

# 启动MQTT Estimator节点
gnome-terminal --tab --title="MQTT Estimator" -- bash -c "sleep 9; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_mqtt mqtt_estimator.launch.py; exec bash"

# 启动MQTT PX4 MISSION任务节点
gnome-terminal --tab --title="MQTT Mission" -- bash -c "sleep 10; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_mqtt mqtt_mission.launch.py; exec bash"

# 启动MQTT 舵机控制节点
gnome-terminal --tab --title="MQTT Servo" -- bash -c "sleep 11; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_mqtt mqtt_servo_control.launch.py; exec bash"

# 启动MQTT Home节点
gnome-terminal --tab --title="MQTT Home" -- bash -c "sleep 12; cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch px4_mqtt mqtt_home_control.launch.py; exec bash"
