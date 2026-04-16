#!/bin/bash

# 启动多个PX4 SITL仿真实例
gnome-terminal --tab --title="PX4 UAV1" -- bash -c "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1; exec bash"
gnome-terminal --tab --title="PX4 UAV2" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="5,10" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2; exec bash"
gnome-terminal --tab --title="PX4 UAV3" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-5,10" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3; exec bash"
gnome-terminal --tab --title="PX4 UAV4" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 4; exec bash"
gnome-terminal --tab --title="PX4 UAV5" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 5; exec bash"
gnome-terminal --tab --title="PX4 UAV6" -- bash -c "cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-10,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 6; exec bash"
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
# gnome-terminal --tab --title="PX4_Multi" -- bash -c "cd ~/px4_ros2_sdk_sitl && source install/setup.bash && ros2 launch uav_first_order_controller multi_uav_launch.py; exec bash"