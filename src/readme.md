# PX4 MQTT Bridge 文档

1. 概述
PX4 MQTT Bridge是一个ROS2软件包，用于在PX4飞控系统和MQTT服务器之间建立双向通信桥梁。该包提供四个核心节点，实现状态数据上传和控制命令下发的完整功能。

1.1 核心功能

    状态监控: 将PX4飞控的状态数据（位置、姿态、电池、GPS等）实时发布到MQTT服务器
    飞行控制: 通过MQTT接收控制命令并转发到PX4飞控
    任务管理: 通过MQTT接收航线任务并管理任务执行状态
    舵机控制: 通过MQTT接收舵机控制命令并转发到PX4
    返航位置更改：通过MQTT接受返航点位置并转发到PX4
    二维码精准降落：通过MQTT发送飞行控制命令，触发二维码降落

1.2 节点组成

    节点名称                                          功能描述
mqtt_estimator                                  PX4状态数据桥接到MQTT
	
mqtt_control                                    MQTT控制命令转发到PX4
	
mqtt_mission                                    航线任务管理和状态反馈

mqtt_servo_control                              舵机控制命令转发

mqtt_home_control                               返航点位置更改


2. 系统架构

2.1 整体架构图

MQTT服务器 ↔ PX4 MQTT Bridge ↔ ROS2中间节点 ↔ PX4飞控

2.2 数据流详细说明
状态数据流（mqtt_estimator）

PX4话题 → px4_estimator → /px4_estimator/state → mqtt_estimator → MQTT服务器

控制命令流（mqtt_control）

MQTT服务器 → mqtt_control → px4_control → PX4飞控

航线任务流（mqtt_mission）

上传: MQTT服务器 → mqtt_mission → /mission/waypoints → mavlink_mission → PX4 dataman存储
执行: MQTT服务器 → mqtt_mission → /px4_mission/trigger → px4_mission → PX4飞控
状态: PX4飞控 → mavlink_mission → px4_mission → mqtt_mission → MQTT服务器

舵机控制命令流（mqtt_servo_control）

MQTT服务器 → mqtt_servo_control → /px4_servo/command → px4_control → PX4飞控

返航点位置更改流（mqtt_home_control）

MQTT服务器 → mqtt_home_control → /px4_home/command → px4_control → PX4飞控

二维码精准降落 （mqtt_control）

MQTT服务器 → mqtt_control → px4_control → 切换到二维码降落模式 → 执行精准降落 → PX4飞控

3. 安装和依赖
3.1 系统依赖

sudo apt-get update
sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev

3.2 ROS2依赖

    rclcpp
    std_msgs
    px4_msgs
    px4_control(需要px4_estimator节点)

3.3 编译安装

colcon build --packages-select px4_mqtt
source install/setup.bash

4. 配置说明
4.1 配置文件结构
所有配置文件位于 config/目录下：

    mqtt_estimator.yaml- 状态估计器配置
    mqtt_control.yaml- 控制命令配置
    mqtt_mission.yaml- 任务管理配置
    mqtt_servo.yaml- 舵机控制配置
    mqtt_home_control.yaml- 返航点位置更改配置

4.2 通用配置参数
所有节点共享以下基本参数：
参数                                                说明                                                默认值
uav_name                                         无人机标识                                             uav1

mqtt_broker                                   MQTT服务器地址                                      tcp://localhost:1883

mqtt_username                                   MQTT用户名                                              空

mqtt_password                                   MQTT密码                                                空

4.3 各节点专用配置
mqtt_estimator专用参数
        参数                                        说明                                               默认值
mqtt_state_topic                                 状态主题前缀                                      uavcontrol/state/

mqtt_control专用参数
        参数                                        说明                                                默认值
mqtt_command_topic                               命令主题前缀                                      uavcontrol/command/

mqtt_mission专用参数
        参数                                        说明                                                 默认值
mqtt_mission_topic                               任务主题前缀                                      uavmission/waypoints/
mqtt_status_topic                                任务状态主题前缀                                    uavmission/status/

mqtt_servo_control专用参数
        参数                                        说明                                                  默认值
mqtt_servo_topic                                舵机主题前缀                                          uavcontrol/servo/

mqtt_home_control专用参数                            说明                                                  默认值
        参数                                    返航位置主题前缀                                       uavcontrol/home/

4.4 配置文件示例
mqtt_estimator.yaml

mqtt_estimator:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_state_topic: "uavcontrol/state/"

mqtt_control.yaml

mqtt_control:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_command_topic: "uavcontrol/command/"

mqtt_mission.yaml

mqtt_mission:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_mission_topic: "uavmission/waypoints/"
    mqtt_status_topic: "uavmission/status/"

mqtt_servo.yaml

mqtt_servo:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_servo_topic: "uavcontrol/servo/"

mqtt_home_control.yaml

mqtt_home:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_servo_topic: "uavcontrol/home/"

5. 使用方法
5.1 启动方式
方法1：使用YAML配置文件（推荐）

# 分别启动各节点
ros2 launch px4_mqtt mqtt_estimator.launch.py
ros2 launch px4_mqtt mqtt_control.launch.py  
ros2 launch px4_mqtt mqtt_mission.launch.py
ros2 launch px4_mqtt mqtt_servo_control.launch.py
ros2 launch px4_mqtt mqtt_home_control.launch.py

方法2：使用launch参数覆盖配置

ros2 launch px4_mqtt mqtt_estimator.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.

方法3：直接运行节点

ros2 run px4_mqtt mqtt_estimator \
    --ros-args \
    -p uav_name:=uav1 \
    -p mqtt_broker:=tcp://118.195.156.74:5704 \
    -p mqtt_username:=uav1 \
    -p mqtt_password:=Aa123456.

5.2 完整的启动流程

# 终端1：启动PX4基础控制节点
ros2 launch px4_control px4_control.launch.py

# 终端2：启动PX4状态估计节点
ros2 launch px4_control px4_estimator.launch.py

# 终端3：启动PX4返航点位置节点
ros2 launch px4_control px4_home_controller.launch.py

# 终端4：启动PX4任务系统节点
ros2 launch px4_control px4_mission.launch.py

# 终端5：启动PX4舵机控制节点
ros2 launch px4_control px4_servo_controller.launch.py

# 终端6：启动MQTT PX4控制节点
ros2 launch px4_mqtt mqtt_control.launch.py

# 终端7：启动MQTT PX4 estimator估计节点
ros2 launch px4_mqtt mqtt_estimator.launch.py

# 终端8：启动MQTT PX4 MISSION任务节点
ros2 launch px4_mqtt mqtt_mission.launch.py

# 终端9：启动MQTT PX4 舵机控制节点
ros2 launch px4_mqtt mqtt_servo_control.launch.py

# 终端10：启动MQTT PX4 返航点位置更改节点
ros2 launch px4_mqtt mqtt_home_control.launch.py


6. MQTT主题和消息格式
6.1 主题命名规则
所有主题都遵循以下格式：

{主题前缀}{无人机名称}

例如：

    状态主题：uavcontrol/state/uav1
    命令主题：uavcontrol/command/uav1
    任务主题：uavmission/waypoints/uav1
    舵机主题：uavcontrol/servo/uav1
    返航点位置主题：uavcontrol/home/uav1

6.2 状态消息格式（mqtt_estimator发布）

{
  "uav_id": "uav1",                 无人机标识
  "uav_name": "uav1",
  "armed": false,                   是否解锁（true/false）
  "arming_state": 1,                解锁状态（0=初始化, 1=未解锁, 2=已解锁）
  "nav_state": 4,                   导航状态
  "battery_v": 12.6,                电池电压（V）
  "battery_b": 85.5,                电池电量百分比（%）
  "battery_current": 2.3,           电池电流（A）
  "position": [1.2, 3.4, 5.6],      本地位置 [x, y, z]（米）
  "velocity": [0.1, 0.2, 0.0],      速度 [vx, vy, vz]（米/秒）
  "dist_bottom":2.865105            离地高度（米）
  "latitude": 39.1234567,           GPS位置（度，米）
  "longitude": 116.1234567,         GPS位置（度，米）
  "altitude": 100.5,                GPS位置（度，米）
  "attitude_rpy": [1.2, 3.4, 45.6], 姿态欧拉角（度）[roll, pitch, yaw]
  "attitude": [0.021, 0.059, 0.796],姿态欧拉角（弧度）[roll, pitch, yaw]
  "gps_status": 3,    GPS定位类型（0=无GPS, 1=无定位, 2=2D定位, 3=3D定位）
  "gps_satellites": 12,             GPS卫星数量
  "gps_speed": 0.0,                 GPS速度（米/秒）
  "control_mode": {                 控制模式标志
    "manual": false,
    "position": true,
    "offboard": false,
    "attitude": false
  },
  "estimator": {                    估计器状态标志
    "gps_ok": true,
    "tilt_align": true,
    "yaw_align": true,
    "baro_hgt": true,
    "in_air": false
  },
  "failsafe": {                     故障保护标志
    "global_pos_invalid": false,
    "local_pos_invalid": false,
    "attitude_invalid": false,
    "battery_unhealthy": false,
    "battery_warning": 0,
    "manual_ctrl_lost": false,
    "gcs_connection_lost": false
  },
    "quaternion": [                 四元数
      0.7041481137275696,
      0.0018249467248097062,
      0.004842985421419144,
      0.7100343108177185
    ],
  "imu": {                          IMU数据（陀螺仪和加速度计）
    "gyro": [0.001, 0.002, 0.003],
    "accel": [0.1, 0.2, 9.8]
  },
  "timestamp": 1234567890123456789
}

6.3 控制命令格式（mqtt_control接收）

基本命令

{"command": "arm"}
{"command": "disarm"}
{"command": "arm_and_takeoff"}

模式切换

{"command": "set_mode_offboard"}
{"command": "set_mode_land"}
{"command": "set_mode_hold"}
{"command": "set_mode_manual"}
{"command": "set_mode_altitude"}
{"command": "set_mode_position"} 
{"command": "set_mode_acro"}
{"command": "set_mode_stabilized"}
{"command": "set_mode_position_orbit"}
{"command": "set_mode_auto_mission"}
{"command": "set_mode_auto_loiter"}
{"command": "set_mode_auto_rtl"}
{"command": "set_mode_auto_precland"}
{"command": "set_mode_auto_vtol_takeoff"}
{"command": "set_mode_external1"}
{"command": "set_mode_external2"}
{"command": "set_mode_external3"}
{"command": "set_mode_external4"}


位置控制

{
  "command": "position",
  "x": 0.0,
  "y": 0.0,
  "z": -5.0,
  "yaw": 0.0     // 偏航角 弧度
}

速度控制

{
  "command": "velocity_body", 
  "vx": 1.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw_rate": 0.0     // 偏航速率  弧度每秒
}

舵机控制

{
  "actuators": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
  "actuator_index": 0,
  "send_frequency": 10.0,
  "update": true
}

返航点位置更改

{
  "command": "set_home", 
  "use_current": false,
  "lat": 47.399526,
  "lon": 8.542604,
  "alt": 0.0,
  "update": true
}

6.4 航线任务格式（mqtt_mission接收）
数组格式（推荐）

{
  "waypoints": [
    {
      "frame": 3,        // 航点坐标系（3=GLOBAL_REL_ALT 全球相对高度，ArduPilot 标配）
      "command": 16		//普通航点命令
      "lat": 47.397767,		//航点纬度
      "lon": 8.545508,		//航点经度
      "alt": 5.0,		//航点高度（相对高度，单位：米）
      "param1": 0.0,		//停留时间（Hold time，秒）
      "param2": 1.0,		//接受半径
      "param3": 0.0,		//通过半径
      "param4": null		//航向角（默认发0.0,飞控自动对准航线）
    },
    {
      "frame": 3,
      "command": 16,
      "lat": 47.398410,
      "lon": 8.545549,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": null
    },
    {
      "frame": 3,
      "command": 16,
      "lat": 47.398353,
      "lon": 8.546756,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": null
    },
    {
      "frame": 3,
      "command": 16,
      "lat": 47.397390,
      "lon": 8.546801,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": null
    }
  ],
  "auto_trigger": false
}
说明： "auto_trigger": false(立即执行航线)
      "auto_trigger": true (不执行航线，仅上传航线)

6.5 任务状态格式（mqtt_mission发布）

{
  "uav_name": "uav1",
  "state": "MISSION_ACTIVE",
  "mission_triggered": true,
  "armed": true,
  "arming_state": 2,
  "nav_state": 3,
  "total_waypoints": 3,
  "current_waypoint": 2,
  "last_reached_waypoint": 1,
  "mission_count_received": true,
  "position": {
    "lat": 47.397742,
    "lon": 8.545594,
    "alt": 5.0
  },
  "timestamp": 1234567890123456789
}

7. ROS2话题接口
7.1 mqtt_estimator话题

    订阅: /px4_estimator/state(std_msgs/String) - PX4状态数据

7.2 mqtt_control话题

    发布: /px4_control/vehicle_command(px4_msgs/VehicleCommand) - 车辆命令
    发布: /px4_control/trajectory_setpoint_command(px4_msgs/TrajectorySetpoint) - 轨迹设定点

7.3 mqtt_mission话题

    发布: /mission/waypoints(std_msgs/Float32MultiArray) - 航点数据
    发布: /px4_mission/trigger(std_msgs/Bool) - 任务触发信号
    订阅: /px4_mission/state(std_msgs/String) - 任务状态

7.4 mqtt_servo_control话题

    发布: /px4_servo/command(px4_msgs/ServoCommand) - 舵机命令

8. 使用示例
8.1 使用mosquitto命令行工具
发送解锁命令

mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "arm"}'

发送位置控制命令

mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "position", "x": 0.0, "y": 0.0, "z": -5.0, "yaw": 0.0}'

发送航线任务

mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavmission/waypoints/uav1" \
  -m '{
    "waypoints": [
      [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
      [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
    ],
    "auto_trigger": true
  }'

发送舵机命令

mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/servo/uav1" \
  -m '{"actuators":[0.1,0.2,0.3,0,0,0],"actuator_index":0,"send_frequency":10.0,"update":true}'

订阅状态信息

mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/state/uav1"

8.2 Python示例代码

import paho.mqtt.client as mqtt
import json

class PX4MQTTController:
    def __init__(self, broker, port, username, password, uav_name):
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.connect(broker, port, 60)
        self.uav_name = uav_name
        
    def send_command(self, command, **kwargs):
        """发送控制命令"""
        message = {"command": command}
        message.update(kwargs)
        
        topic = f"uavcontrol/command/{self.uav_name}"
        self.client.publish(topic, json.dumps(message))
        print(f"Sent command: {command}")
    
    def send_mission(self, waypoints, auto_trigger=True):
        """发送航线任务"""
        message = {
            "waypoints": waypoints,
            "auto_trigger": auto_trigger
        }
        
        topic = f"uavmission/waypoints/{self.uav_name}"
        self.client.publish(topic, json.dumps(message))
        print(f"Sent mission with {len(waypoints)} waypoints")
    
    def disconnect(self):
        self.client.disconnect()

# 使用示例
controller = PX4MQTTController(
    broker="118.195.156.74",
    port=5704,
    username="uav1", 
    password="Aa123456.",
    uav_name="uav1"
)

# 解锁
controller.send_command("arm")

# 切换到OFFBOARD模式
controller.send_command("set_mode_offboard")

# 发送位置命令
controller.send_command("position", x=0.0, y=0.0, z=-5.0, yaw=0.0)

# 发送航线任务
waypoints = [
    [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
    [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
]
controller.send_mission(waypoints)

controller.disconnect()

9. 故障排除
9.1 常见问题及解决方案
MQTT连接失败

# 检查网络连接
ping 118.195.156.74

# 检查MQTT服务端口
telnet 118.195.156.74 5704

# 检查节点日志
ros2 launch px4_mqtt mqtt_estimator.launch.py

没有数据发布到MQTT

# 检查前置节点是否运行
ros2 node list | grep px4_estimator

# 检查ROS2话题数据
ros2 topic echo /px4_estimator/state

# 检查话题列表
ros2 topic list | grep px4_estimator

配置文件问题

# 检查源目录配置文件（优先使用）
cat ~/px4_sitl/px4_ros2_ws/src/px4_mqtt/config/mqtt_estimator.yaml

# 检查安装目录配置文件（备用）
cat ~/px4_sitl/px4_ros2_ws/install/px4_mqtt/share/px4_mqtt/config/mqtt_estimator.yaml

# 检查YAML格式
yamllint ~/px4_sitl/px4_ros2_ws/src/px4_mqtt/config/mqtt_estimator.yaml

编译错误

# 检查依赖
dpkg -l | grep -E "paho-mqtt|nlohmann"

# 重新安装依赖
sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev

# 清理缓存重新编译
cd ~/px4_sitl/px4_ros2_ws
colcon build --packages-select px4_mqtt --cmake-clean-cache
source install/setup.bash

9.2 调试工具
ROS2诊断命令

# 查看节点状态
ros2 node list
ros2 node info /mqtt_estimator

# 查看话题状态
ros2 topic list
ros2 topic info /px4_estimator/state
ros2 topic echo /px4_estimator/state

# 查看服务状态
ros2 service list

# 查看参数
ros2 param list
ros2 param get /mqtt_estimator uav_name

MQTT诊断命令

# 订阅所有主题（调试用）
mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. -t "#" -v

# 测试连接
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. -t "test" -m "hello"

10. 注意事项
10.1 重要安全提示

    测试环境: 首次使用请在仿真环境中测试所有功能
    安全距离: 实际飞行时确保安全距离和应急措施
    通信安全: 使用TLS加密连接生产环境
    权限管理: 严格控制MQTT访问权限

10.2 技术注意事项
坐标系说明

    所有位置和速度使用NED（North-East-Down）坐标系
    z为负值表示高度（例如：z: -5.0表示5米高度）
    经纬度使用WGS84坐标系

启动顺序要求

# 必须按顺序启动
1. px4_estimator (状态估计)
2. mqtt_estimator (状态桥接)
3. px4_control (控制基础)
4. mqtt_control (控制桥接) 
5. px4_mission (任务基础)
6. mqtt_mission (任务桥接)

配置优先级

    Launch命令行参数（最高优先级）
    源目录配置文件（src/px4_mqtt/config/）
    安装目录配置文件（install/px4_mqtt/share/px4_mqtt/config/）

10.3 性能优化建议

    QoS配置: 根据需求调整QoS级别（状态数据可用BestEffort，关键命令用Reliable）
    发布频率: 调整状态发布频率以减少网络负载
    消息大小: 优化JSON结构减少数据传输量
    连接池: 高频应用考虑使用连接池

11. 扩展开发
11.1 自定义消息格式
可以通过修改相应节点的JSON序列化/反序列化逻辑来自定义消息格式。
11.2 添加新命令类型
在mqtt_control节点中添加新的命令解析逻辑，并在px4_control中实现对应的处理逻辑。
11.3 集成其他传感器
可以通过创建新的ROS2节点来集成其他传感器数据，并按照相同模式桥接到MQTT。
12. 技术支持
12.1 文档更新
本文档会随代码更新，最新版本请参考项目README文件。
12.2 问题反馈
遇到问题时请提供：

    ROS2版本信息
    PX4版本信息
    错误日志和配置信息
    复现步骤

12.3 社区支持

    GitHub Issues: 项目问题跟踪
    PX4 Discourse: 社区讨论
    ROS Answers: ROS相关问题

版权声明: 本文档遵循BSD 3-Clause许可证
最后更新: 2026年2月3日

source yolov8_env/bin/activate
deactivate


