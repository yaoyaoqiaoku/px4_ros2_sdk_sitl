#!/usr/bin/env python
"""
MQTT舵机控制节点的启动文件。
订阅MQTT代理服务器的舵机控制指令,并将指令发布到ROS2系统中。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 声明启动参数（可覆盖YAML配置文件中的参数）
    uav_name_arg = DeclareLaunchArgument(
        'uav_name',
        default_value='',
        description='无人机名称/ID(若提供则覆盖YAML配置文件中的对应值)'
    )
    
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='',
        description='MQTT代理服务器地址(若提供则覆盖YAML配置文件中的对应值)'
    )
    
    mqtt_username_arg = DeclareLaunchArgument(
        'mqtt_username',
        default_value='',
        description='MQTT用户名(若提供则覆盖YAML配置文件中的对应值)'
    )
    
    mqtt_password_arg = DeclareLaunchArgument(
        'mqtt_password',
        default_value='',
        description='MQTT密码(若提供则覆盖YAML配置文件中的对应值)'
    )
    
    mqtt_servo_topic_arg = DeclareLaunchArgument(
        'mqtt_servo_topic',
        default_value='',
        description='MQTT舵机话题前缀(若提供则覆盖YAML配置文件中的对应值)'
    )

    return LaunchDescription([
        uav_name_arg,
        mqtt_broker_arg,
        mqtt_username_arg,
        mqtt_password_arg,
        mqtt_servo_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    # 获取功能包的共享目录（安装后的路径）
    pkg_share = FindPackageShare(package='px4_mqtt').find('px4_mqtt')
    install_config_file = os.path.join(pkg_share, 'config', 'mqtt_servo_control.yaml')
    
    # 尝试查找源码目录下的配置文件（将安装路径替换为源码路径）
    source_config_file = None
    if 'install' in pkg_share:
        # 将 install/px4_mqtt/share/px4_mqtt 路径替换为 /src/px4_mqtt
        src_dir = pkg_share.replace('/install/px4_mqtt/share/px4_mqtt', '/src/px4_mqtt')
        source_config_file = os.path.join(src_dir, 'config', 'mqtt_servo_control.yaml')
    
    # 优先使用源码目录下的配置文件（如果存在），否则使用安装后的配置文件
    if source_config_file and os.path.exists(source_config_file):
        config_file = source_config_file
    else:
        config_file = install_config_file

    # 获取启动参数的值
    uav_name = LaunchConfiguration('uav_name').perform(context)
    mqtt_broker = LaunchConfiguration('mqtt_broker').perform(context)
    mqtt_username = LaunchConfiguration('mqtt_username').perform(context)
    mqtt_password = LaunchConfiguration('mqtt_password').perform(context)
    mqtt_servo_topic = LaunchConfiguration('mqtt_servo_topic').perform(context)

    # 构建参数列表 - 先加载YAML配置文件中的参数
    parameters = [config_file]
    
    # 仅当启动参数有值（非空）时，才添加覆盖参数
    override_params = {}
    if uav_name and uav_name.strip():
        override_params['uav_name'] = uav_name
    if mqtt_broker and mqtt_broker.strip():
        override_params['mqtt_broker'] = mqtt_broker
    if mqtt_username and mqtt_username.strip():
        override_params['mqtt_username'] = mqtt_username
    if mqtt_password and mqtt_password.strip():
        override_params['mqtt_password'] = mqtt_password
    if mqtt_servo_topic and mqtt_servo_topic.strip():
        override_params['mqtt_servo_topic'] = mqtt_servo_topic
    
    if override_params:
        parameters.append(override_params)

    mqtt_servo_node = Node(
        package='px4_mqtt',          # 功能包名称
        executable='mqtt_servo_control',  # 可执行文件名称
        name='mqtt_servo_control',   # 节点名称
        output='screen',             # 输出打印到终端
        parameters=parameters,       # 节点参数
        # 如果需要指定命名空间，可以添加 namespace 参数
        # namespace=uav_name,
    )

    return [mqtt_servo_node]
