#!/usr/bin/env python3

"""
Launch file for PX4 Servo Controller node.
Controls servos by publishing VehicleCommand messages to PX4.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_control',
            executable='px4_servo_controller',  # 与CMakeLists.txt中设置的可执行文件名对应
            name='px4_servo_controller_node',    # 节点名称，与代码中默认名称一致
            output='screen',
            parameters=[
                # 可在此处添加默认参数配置，示例：
                # {'actuator_1': 0.0},
                # {'actuator_2': 0.0},
                # {'send_frequency': 10.0}
            ]
        )
    ])
