# PX4 MQTT Bridge

ROS2包，用于将PX4飞控的状态数据桥接到MQTT服务器，以及通过MQTT接收控制命令。

## 概述

本包包含四个节点：

1. **`mqtt_estimator`**: 订阅`px4_estimator`节点发布的状态话题，将PX4飞控的状态数据（位置、姿态、电池、GPS等）转换为JSON格式并发布到MQTT服务器。

2. **`mqtt_control`**: 订阅MQTT服务器上的控制命令主题，接收JSON格式的控制命令并转发到PX4飞控。

3. **`mqtt_mission`**: 订阅MQTT服务器上的航线任务主题，接收JSON格式的航线任务并转发到ROS2任务系统；订阅`px4_mission`节点发布的任务状态并转发到MQTT服务器。

4. **`mqtt_servo_control`**: 通过 MQTT 接收舵机（servo）命令并在 ROS2 中发布对应的 `px4_msgs::msg::ServoCommand`，供 `px4_control` 或其它节点转发到 PX4 或本地处理。

## 架构

### 状态数据流（mqtt_estimator）

```
PX4话题 → px4_estimator → /px4_estimator/state → mqtt_estimator → MQTT服务器
```

### 控制命令流（mqtt_control）

```
MQTT服务器 → mqtt_control → px4_control → PX4飞控
```

### 航线任务流（mqtt_mission）

```
MQTT服务器 → mqtt_mission → /mission/waypoints → mavlink_mission → MAVLink协议 → PX4 dataman存储
MQTT服务器 → mqtt_mission → /px4_mission/trigger → px4_mission → PX4飞控执行
PX4飞控 → MAVLink消息 → mavlink_mission → /mission/count, /mission/current_waypoint → px4_mission
px4_mission → /px4_mission/state → mqtt_mission → MQTT服务器
```

### 舵机控制命令流

```
MQTT服务器 → mqtt_servo_control → /px4_servo/command (px4_msgs::msg::ServoCommand) → px4_control → PX4飞控
```

`mqtt_servo_control` 节点订阅 MQTT 上的舵机命令主题（例如 `{mqtt_servo_topic}{uav_name}`），把解析后的命令发布为 ROS2 消息 `/px4_servo/command`（类型为 `px4_msgs::msg::ServoCommand`）。`px4_control` 中的 `PX4ServoController` 或其他适配器再决定如何把该消息转发到 PX4（比如通过 MAV_CMD_DO_SET_SERVO 或映射到 actuator_controls）。

`mqtt_estimator`节点订阅`px4_estimator`发布的状态话题，避免重复订阅PX4话题。

`mqtt_mission`节点与`px4_control`包中的`px4_mission`节点集成，架构与`mqtt_estimator`和`mqtt_control`一致：
- 通过MQTT接收航线任务，发布到`/mission/waypoints`（供`mavlink_mission`通过MAVLink Mission协议上传到PX4的`dataman`存储）和`/px4_mission/trigger`（供`px4_mission`执行任务`）
- 订阅`px4_mission`发布的状态话题`/px4_mission/state`，转发到MQTT服务器
- 任务进度由`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用

备注：关于如何在不同 PX4 配置中使 `ServoCommand` 生效，请参考下文“MQTT Servo 节点”节和 TODO 列表中的 actuator_controls 后备实现。
- 通过MQTT接收航线任务，发布到`/mission/waypoints`（供`mavlink_mission`通过MAVLink Mission协议上传到PX4的`dataman`存储）和`/px4_mission/trigger`（供`px4_mission`执行任务）
- 订阅`px4_mission`发布的状态话题`/px4_mission/state`，转发到MQTT服务器
- 任务进度由`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用

`mqtt_control`节点发布命令到`/px4_servo/command`

## MQTT Servo 节点（mqtt_servo_control）

`mqtt_servo_control` 节点用于通过 MQTT 接收舵机（servo）命令，并在 ROS2 中发布 `px4_msgs::msg::ServoCommand` 消息，供 `px4_control` 或其他订阅者转发到 PX4 或本地执行。

主要功能：
- 订阅 MQTT 上的舵机命令主题（示例：`{mqtt_servo_topic}{uav_name}`）
- 将 JSON 格式的舵机命令解析为 `px4_msgs::msg::ServoCommand` 并发布到 ROS 话题 `/px4_servo/command`
- 提供参数化配置（`src/px4_mqtt/config/mqtt_servo.yaml`）以便于不同环境下复用

配置说明
---------
配置文件路径：`src/px4_mqtt/config/mqtt_servo.yaml`（开发时优先读取源目录中的配置；安装后会复制到 `install/.../config/`）。

参数（yaml 中的 `mqtt_servo` 下的 `ros__parameters`）：

| 参数 | 说明 | 默认 |
|------|------|------|
| `uav_name` | 无人机名称/ID（用于拼接 MQTT topic 和 client id） | `uav1` |
| `mqtt_broker` | MQTT 服务器地址（支持 `tcp://host:port` 或 `mqtt://host:port`） | `tcp://118.195.156.74:5704` |
| `mqtt_username` | MQTT 用户名（如需认证） | `uav1` |
| `mqtt_password` | MQTT 密码（如需认证） | `Aa123456.` |
| `mqtt_servo_topic` | MQTT 舵机命令主题前缀（最终主题 `{mqtt_servo_topic}{uav_name}`） | `uavcontrol/servo/` |

示例（`src/px4_mqtt/config/mqtt_servo.yaml`）：

```yaml
mqtt_servo:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_servo_topic: "uavcontrol/servo/"
```

MQTT 消息格式（payload）
-------------------------
节点期望接收 JSON 格式的舵机命令，示例：

```json
{
  "actuators": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
  "actuator_index": 0,
  "send_frequency": 10.0,
  "update": true
}
```

字段说明：
- `actuators`: 浮点数组，表示各舵机的输出（含义由上层决定，归一化值）
- `actuator_index`: 起始通道索引（uint8）
- `send_frequency`: 希望的发送/维持频率（Hz）
- `update`: 是否立即生效（bool）

运行与测试（快速开始）
------------------------

1. 启动节点（使用参数文件）

```bash
source install/setup.bash
ros2 run px4_mqtt mqtt_servo_control --ros-args --params-file src/px4_mqtt/config/mqtt_servo.yaml
```

2. 使用 `mosquitto_pub` 发送测试消息：

```bash
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/servo/uav1" \
  -m '{"actuators":[0.1,0.2,0.3,0,0,0],"actuator_index":0,"send_frequency":10.0,"update":true}'
```

3. 在 ROS 端查看发布（如果节点解析并发布成功）：

```bash
ros2 topic echo /px4_servo/command
```

调试与排查要点
---------------
- YAML 解析错误：如果以 `--params-file` 启动时报 `Failed to parse global arguments` 或 `RCLInvalidROSArgsError`，请检查 YAML 是否使用了制表符（tab）或包含不可见字符，确保文件以空格缩进并使用 UTF-8。可用 `cat -A`、`hexdump -C` 检查。
- 未收到 ROS 消息：检查节点日志，确认 MQTT 连接建立并订阅了正确 topic（`{mqtt_servo_topic}{uav_name}`）；确认 MQTT broker 可达且消息已到达。
- 消息字段或长度不匹配：确保 `actuators` 数组长度与 `px4_control` / PX4 侧的期望匹配，或在上游做截断/扩展处理。

与 PX4 集成注意事项
--------------------
- `mqtt_servo_control` 只负责把 MQTT payload 转为 `px4_msgs::msg::ServoCommand` 并发布到 ROS 话题，实际控制 PX4 的方式取决于 `px4_control` 的实现：
  - 可以把 `ServoCommand` 转为 MAV_CMD_DO_SET_SERVO（如果 PX4 支持）
  - 或者把值映射到 `actuator_controls` 话题（更通用，但需要映射规则与通道配置）
- 在不同 PX4 固件或配置中，某些命令可能被拒绝或忽略（例如安全模式、没有外部控制许可等），启动前请确认 PX4 的模式与权限设置。

开发者说明
--------------
- 源配置路径：`src/px4_mqtt/config/mqtt_servo.yaml`（开发时优先读取）
- 依赖：同其它 mqtt 节点，需安装 `libpaho-mqttcpp-dev` 和 `nlohmann-json3-dev`，并在 `package.xml` 中列出 `rclcpp`、`px4_msgs` 等
- 建议改进：添加 `launch/mqtt_servo.launch.py`；实现 actuator_controls 作为后备并在 README 明确映射规则（已列入 TODO）

## 依赖

### 系统依赖

```bash
sudo apt-get update
sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev
```

### ROS2依赖

- `rclcpp`
- `std_msgs`
- `px4_msgs`
- `px4_control` (需要`px4_estimator`节点运行，用于`mqtt_estimator`)

## 编译

```bash
cd ~/px4_sitl/px4_ros2_ws
colcon build --packages-select px4_mqtt
source install/setup.bash
```

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_estimator.yaml`，包含以下参数：

**重要提示：** 
- Launch文件会**优先读取源目录**（`src/px4_mqtt/config/`）中的配置文件
- 修改配置文件后**无需重新编译**，直接运行launch文件即可生效
- 如果源目录配置文件不存在，会自动使用安装目录（`install/px4_mqtt/share/px4_mqtt/config/`）中的配置文件

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_state_topic` | MQTT状态主题前缀 | `uavcontrol/state/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_estimator.yaml`：

```yaml
mqtt_estimator:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_state_topic: "uavcontrol/state/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_estimator.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_estimator.launch.py
```

**优势：** 修改配置文件后无需重新编译安装，直接运行launch文件即可生效（类似ROS1的行为）

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_estimator.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

### 方法3：直接运行节点

```bash
ros2 run px4_mqtt mqtt_estimator \
    --ros-args \
    -p uav_name:=uav1 \
    -p mqtt_broker:=tcp://118.195.156.74:5704 \
    -p mqtt_username:=uav1 \
    -p mqtt_password:=Aa123456. \
    -p mqtt_state_topic:=uavcontrol/state/
```

## 前提条件

在启动`mqtt_estimator`之前，**必须先启动`px4_estimator`节点**：

```bash
# 终端1：启动px4_estimator
ros2 launch px4_control px4_estimator.launch.py

# 终端2：启动mqtt_estimator
ros2 launch px4_mqtt mqtt_estimator.launch.py
```

## PX4_Home_Controller节点

json 格式

{
  "command": "set_home", 
  "use_current": false,
  "lat": 39.9042,
  "lon": 116.4074,
  "alt": 50.0,
  "update": true
}

## ROS2话题

### 订阅的话题

- `/px4_estimator/state` (`std_msgs/msg/String`)
  - 订阅`px4_estimator`节点发布的JSON格式状态数据
  - 发布频率：1Hz

## MQTT主题

状态数据发布到主题：`{mqtt_state_topic}{uav_name}`

**示例：**
- 如果 `mqtt_state_topic` = `uavcontrol/state/`
- 如果 `uav_name` = `uav1`
- 则完整主题 = `uavcontrol/state/uav1`

## JSON消息格式

发布的JSON消息包含以下字段：

```json
{
  "uav_id": "uav1",
  "uav_name": "uav1",
  "armed": false,
  "arming_state": 1,
  "nav_state": 4,
  "battery_v": 12.6,
  "battery_b": 85.5,
  "battery_current": 2.3,
  "position": [1.2, 3.4, 5.6],
  "velocity": [0.1, 0.2, 0.0],
  "latitude": 39.1234567,
  "longitude": 116.1234567,
  "altitude": 100.5,
  "attitude_rpy": [1.2, 3.4, 45.6],
  "attitude": [0.021, 0.059, 0.796],
  "gps_status": 3,
  "gps_satellites": 12,
  "gps_speed": 0.0,
  "control_mode": {
    "manual": false,
    "position": true,
    "offboard": false,
    "attitude": false
  },
  "estimator": {
    "gps_ok": true,
    "tilt_align": true,
    "yaw_align": true,
    "baro_hgt": true,
    "in_air": false
  },
  "failsafe": {
    "global_pos_invalid": false,
    "local_pos_invalid": false,
    "attitude_invalid": false,
    "battery_unhealthy": false,
    "battery_warning": 0,
    "manual_ctrl_lost": false,
    "gcs_connection_lost": false
  },
  "imu": {
    "gyro": [0.001, 0.002, 0.003],
    "accel": [0.1, 0.2, 9.8]
  },
  "timestamp": 1234567890123456789
}
```

### 字段说明

- `uav_id`, `uav_name`: 无人机标识
- `armed`: 是否解锁（true/false）
- `arming_state`: 解锁状态（0=初始化, 1=未解锁, 2=已解锁）
- `nav_state`: 导航状态
- `battery_v`: 电池电压（V）
- `battery_b`: 电池电量百分比（%）
- `battery_current`: 电池电流（A）
- `position`: 本地位置 [x, y, z]（米）
- `velocity`: 速度 [vx, vy, vz]（米/秒）
- `latitude`, `longitude`, `altitude`: GPS位置（度，米）
- `attitude_rpy`: 姿态欧拉角（度）[roll, pitch, yaw]
- `attitude`: 姿态欧拉角（弧度）[roll, pitch, yaw]
- `gps_status`: GPS定位类型（0=无GPS, 1=无定位, 2=2D定位, 3=3D定位）
- `gps_satellites`: GPS卫星数量
- `gps_speed`: GPS速度（米/秒）
- `control_mode`: 控制模式标志
- `estimator`: 估计器状态标志
- `failsafe`: 故障保护标志
- `imu`: IMU数据（陀螺仪和加速度计）
- `timestamp`: 时间戳（纳秒）

## 故障排除

### MQTT连接失败

1. **检查配置文件**（优先检查源目录）
   ```bash
   # 检查源目录配置文件（实际使用的）
   cat ~/px4_sitl/px4_ros2_ws/src/px4_mqtt/config/mqtt_estimator.yaml
   
   # 检查安装目录配置文件（备用）
   cat ~/px4_sitl/px4_ros2_ws/install/px4_mqtt/share/px4_mqtt/config/mqtt_estimator.yaml
   ```

2. **检查MQTT服务器**
   - 确认服务器地址和端口正确
   - 确认用户名和密码正确
   - 测试网络连接：`ping <mqtt_server_ip>`

3. **查看节点日志**
   ```bash
   ros2 launch px4_mqtt mqtt_estimator.launch.py
   ```

### 没有数据发布到MQTT

1. **检查`px4_estimator`是否运行**
   ```bash
   ros2 node list | grep px4_estimator
   ```

2. **检查状态话题是否有数据**
   ```bash
   ros2 topic echo /px4_estimator/state
   ```

3. **检查话题是否存在**
   ```bash
   ros2 topic list | grep px4_estimator
   ```

4. **检查MQTT连接状态**
   - 查看节点日志中的连接信息
   - 确认MQTT服务器是否收到消息

### 编译错误

1. **检查依赖是否安装**
   ```bash
   dpkg -l | grep -E "paho-mqtt|nlohmann"
   ```

2. **重新安装依赖**
   ```bash
   sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev
   ```

3. **清理并重新编译**
   ```bash
   cd ~/px4_sitl/px4_ros2_ws
   colcon build --packages-select px4_mqtt --cmake-clean-cache
   source install/setup.bash
   ```

---

# MQTT Control 节点

## 概述

`mqtt_control`节点订阅MQTT服务器上的控制命令主题，接收JSON格式的控制命令并转发到PX4飞控。

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_control.yaml`，包含以下参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_command_topic` | MQTT命令主题前缀 | `uavcontrol/command/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_control.yaml`：

```yaml
mqtt_control:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_command_topic: "uavcontrol/command/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_control.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_control.launch.py
```

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_control.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

## MQTT主题

控制命令订阅主题：`{mqtt_command_topic}{uav_name}`

**示例：**
- 如果 `mqtt_command_topic` = `uavcontrol/command/`
- 如果 `uav_name` = `uav1`
- 则完整主题 = `uavcontrol/command/uav1`

## 支持的命令

### 基本命令

#### 1. 解锁 (arm)

```json
{
  "command": "arm"
}
```

#### 2. 上锁 (disarm)

```json
{
  "command": "disarm"
}
```

#### 3. 解锁并起飞 (arm_and_takeoff)

```json
{
  "command": "arm_and_takeoff"
}
```

### 模式切换命令

#### 4. 切换到OFFBOARD模式 (set_mode_offboard)

```json
{
  "command": "set_mode_offboard"
}
```

#### 5. 切换到LAND模式 (set_mode_land)

```json
{
  "command": "set_mode_land"
}
```

#### 6. 切换到HOLD模式 (set_mode_hold)

```json
{
  "command": "set_mode_hold"
}
```

### 位置控制命令

#### 7. 位置设定点 (position)

```json
{
  "command": "position",
  "x": 0.0,
  "y": 0.0,
  "z": -5.0,
  "yaw": 0.0
}
```

- `x`, `y`, `z`: 位置坐标（NED坐标系，单位：米）
- `yaw`: 偏航角（可选，默认：0.0，单位：弧度）

**示例：** 悬停在5米高度

```json
{
  "command": "position",
  "x": 0.0,
  "y": 0.0,
  "z": -5.0,
  "yaw": 0.0
}
```

### 速度控制命令

#### 8. 速度设定点 (velocity)

```json
{
  "command": "velocity",
  "vx": 1.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw": 0.0
}
```

- `vx`, `vy`, `vz`: 速度（NED坐标系，单位：米/秒）
- `yaw`: 偏航角（可选，默认：0.0，单位：弧度）

**示例：** 以1 m/s速度向前移动

```json
{
  "command": "velocity",
  "vx": 1.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw": 0.0
}
```
MQTT JSON 示例（典型）
（把下面 JSON 发到你订阅的 MQTT 主题，例如 uavcontrol/command/uav1）

Arm / Disarm

Arm: {"command": "arm"}
Disarm: {"command": "disarm"}
基本模式切换（直接字符串命令）

切到 OFFBOARD（已有实现）： {"command": "set_mode_offboard"}
切到 MANUAL： {"command": "set_mode_manual"}
切到 POSCTL： {"command": "set_mode_posctl"}
切到 ACRO： {"command": "set_mode_acro"}
切到 STABILIZED： {"command": "set_mode_stabilized"}
POSCTL.ORBIT（示例）： {"command": "set_mode_posctl_orbit"}
AUTO 子模式

AUTO.TAKEOFF: {"command": "set_mode_auto_takeoff"}
AUTO.MISSION: {"command": "set_mode_auto_mission"}
AUTO.LOITER: {"command": "set_mode_auto_loiter"}
AUTO.RTL: {"command": "set_mode_auto_rtl"}
AUTO.PRECLAND: {"command": "set_mode_auto_precland"}
AUTO.VTOL_TAKEOFF: {"command": "set_mode_auto_vtol_takeoff"}
EXTERNAL1..EXTERNAL8

EXTERNAL3（示例）： {"command": "set_mode_external3"}
位置 / 速度 setpoint（已有）

Position: {"command":"position", "x":0.0, "y":0.0, "z":-5.0, "yaw":0.0}
Velocity: {"command":"velocity", "vx":1.0, "vy":0.0, "vz":0.0, "yaw":0.0}
设置 6 路 actuator outputs（一次性）

格式（values 长度 6，index 可选）： { "command": "set_actuator_outputs", "values": [0.0, 0.0, 0.5, -0.1, 0.0, 0.0], "index": 0 }
设置单个舵机 PWM（使用 DO_REPEAT_SERVO）

例如把舵机 1 设为 1500us： {"command":"set_servo_pwm", "servo":1, "pwm":1500}

## ROS2话题

### 发布的话题

- `/px4_control/vehicle_command` (`px4_msgs/msg/VehicleCommand`)
  - 发布车辆命令到`px4_control`节点（解锁、上锁、模式切换等）
  - `px4_control`节点会转发给PX4飞控

- `/px4_control/trajectory_setpoint_command` (`px4_msgs/msg/TrajectorySetpoint`)
  - 发布轨迹设定点命令到`px4_control`节点（位置/速度控制）
  - `px4_control`节点会转发给PX4飞控并持续发布以维持offboard模式

## 使用示例

### 使用mosquitto_pub发送命令

```bash
# 解锁
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "arm"}'

# 切换到OFFBOARD模式
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "set_mode_offboard"}'

# 位置控制：悬停在5米高度
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "position", "x": 0.0, "y": 0.0, "z": -5.0, "yaw": 0.0}'

# 速度控制：向前移动1 m/s
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "velocity", "vx": 1.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}'

# 降落
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "set_mode_land"}'

# 上锁
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "disarm"}'
```

### Python示例

```python
import paho.mqtt.client as mqtt
import json

# MQTT配置
broker = "118.195.156.74"
port = 5704
username = "uav1"
password = "Aa123456."
topic = "uavcontrol/command/uav1"

# 创建MQTT客户端
client = mqtt.Client()
client.username_pw_set(username, password)
client.connect(broker, port, 60)

# 发送解锁命令
cmd = {"command": "arm"}
client.publish(topic, json.dumps(cmd))

# 发送位置控制命令
cmd = {
    "command": "position",
    "x": 0.0,
    "y": 0.0,
    "z": -5.0,
    "yaw": 0.0
}
client.publish(topic, json.dumps(cmd))

client.disconnect()
```

## 前提条件

在启动`mqtt_control`之前，**必须先启动`px4_control`节点**（`mqtt_control`通过`px4_control`转发命令）：

```bash
# 终端1：启动px4_control（必需）
ros2 launch px4_control px4_control.launch.py

# 终端2：启动mqtt_control
ros2 launch px4_mqtt mqtt_control.launch.py
```

**注意：** `mqtt_control`节点不直接与PX4飞控通信，所有命令都通过`px4_control`节点转发。

## 注意事项

1. **坐标系**: 所有位置和速度使用NED（North-East-Down）坐标系
   - `z`为负值表示高度（例如：`z: -5.0` 表示5米高度）

---

# MQTT Mission 节点

## 概述

`mqtt_mission`节点订阅MQTT服务器上的航线任务主题，接收JSON格式的航线任务并转发到ROS2任务系统；同时订阅`px4_mission`节点发布的任务状态并转发到MQTT服务器。

**架构说明：**
- 与`mqtt_estimator`和`mqtt_control`的架构一致
- 通过`px4_mission`节点与PX4通信，避免直接与PX4交互
- 航点数据发布到`/mission/waypoints`供`mavlink_mission`上传到PX4
- 任务触发信号发布到`/px4_mission/trigger`供`px4_mission`执行
- 任务状态从`/px4_mission/state`订阅并转发到MQTT

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_mission.yaml`，包含以下参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_mission_topic` | MQTT任务主题前缀 | `uavmission/waypoints/` |
| `mqtt_status_topic` | MQTT状态主题前缀 | `uavmission/status/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_mission.yaml`：

```yaml
mqtt_mission:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_mission_topic: "uavmission/waypoints/"
    mqtt_status_topic: "uavmission/status/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_mission.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_mission.launch.py
```

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_mission.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

## MQTT主题

- **任务订阅主题**：`{mqtt_mission_topic}{uav_name}`
- **状态发布主题**：`{mqtt_status_topic}{uav_name}`

**示例：**
- 如果 `mqtt_mission_topic` = `uavmission/waypoints/`
- 如果 `mqtt_status_topic` = `uavmission/status/`
- 如果 `uav_name` = `uav1`
- 则任务主题 = `uavmission/waypoints/uav1`
- 则状态主题 = `uavmission/status/uav1`

## 支持的MQTT消息格式

### 1. 发送航线任务

#### 格式1：数组格式（推荐）

```json
{
  "waypoints": [
    [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
    [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
    [3, 16, 47.397942, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
  ],
  "auto_trigger": true
}
```

#### 格式2：对象格式

```json
{
  "waypoints": [
    {
      "frame": 3,
      "command": 16,
      "lat": 47.397767,
      "lon": 8.545508,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": 0.0
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
      "param4": 0.0
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
      "param4": 0.0
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
      "param4": 0.0
    }
  ],
  "auto_trigger": false
}
```

**航点参数说明：**
- `frame`: 坐标系类型（0=全局绝对高度, 1=本地NED, 2=任务坐标系, 3=全局相对高度，推荐）
- `command`: MAVLink命令类型（16=NAV_WAYPOINT标准航点）
- `lat`: 纬度（度）
- `lon`: 经度（度）
- `alt`: 高度（米）
- `param1`: 保持时间（秒）
- `param2`: 接受半径（米）
- `param3`: 通过半径（米，0=直接通过）
- `param4`: 偏航角（弧度，0=不改变）

**`auto_trigger`**: 是否自动触发任务执行（默认：`false`）

### 2. 触发任务执行

```json
{
  "trigger": true
}
```

或

```json
{
  "action": "start"
}
```

## 任务状态消息格式

节点会实时转发`px4_mission`发布的任务状态到MQTT（2Hz更新频率）：

```json
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
```

**状态字段说明：**
- `uav_name`: 无人机名称（由`mqtt_mission`添加）
- `state`: 任务执行状态（`IDLE`, `ARMING`, `MISSION_ACTIVE`, `MISSION_COMPLETE`, `ERROR`）
- `mission_triggered`: 任务是否已触发
- `armed`: 是否已解锁
- `arming_state`: 解锁状态（0=初始化, 1=未解锁, 2=已解锁）
- `nav_state`: 导航状态（3=AUTO_MISSION, 4=AUTO_LOITER/HOLD等）
- `total_waypoints`: 总航点数（从`mavlink_mission`的`/mission/count`话题获取）
- `current_waypoint`: 当前航点索引（从0开始，从`mavlink_mission`的`/mission/current_waypoint`话题获取）
- `last_reached_waypoint`: 最后到达的航点索引（从0开始，如果未到达任何航点则此字段不存在）
- `mission_count_received`: 是否已收到航点总数（从`/mission/count`话题）
- `position`: GPS位置信息（如果GPS可用则包含此字段，包含`lat`, `lon`, `alt`）
- `timestamp`: 时间戳（纳秒）

**注意：**
- 任务进度通过`mavlink_mission`节点监听MAVLink消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用
- `px4_mission`节点订阅这些话题来跟踪任务进度，不再依赖`mission_result` uORB话题（该话题在某些PX4配置中可能不可用）

## 完整启动流程

在启动`mqtt_mission`之前，**必须先启动任务系统**：

```bash
# 终端1：启动任务系统（必需）
# 这会启动 mavlink_mission.py 和 px4_mission 节点
ros2 launch px4_control px4_mission.launch.py

# 终端2：启动mqtt_mission
ros2 launch px4_mqtt mqtt_mission.launch.py
```

**启动顺序说明：**
1. 首先启动`px4_mission.launch.py`，这会启动：
   - `mavlink_mission.py`: 负责通过MAVLink Mission协议上传航点到PX4的`dataman`存储，监听MAVLink消息并发布任务进度到`/mission/count`和`/mission/current_waypoint`话题
   - `px4_mission`: 负责执行任务，订阅`/mission/count`和`/mission/current_waypoint`跟踪进度，并发布状态到`/px4_mission/state`
2. 然后启动`mqtt_mission.launch.py`，这会启动：
   - `mqtt_mission`: 订阅MQTT任务并发布到ROS2话题（`/mission/waypoints`和`/px4_mission/trigger`），订阅`/px4_mission/state`并转发到MQTT

**注意：** 
- `mqtt_mission`节点与`px4_control`包中的`px4_mission`节点集成，架构与`mqtt_estimator`和`mqtt_control`一致
- 航点数据通过`/mission/waypoints`话题发送给`mavlink_mission`节点，通过MAVLink协议上传到PX4的`dataman`存储
- 任务执行通过`/px4_mission/trigger`话题触发`px4_mission`节点
- 任务状态从`/px4_mission/state`话题订阅并转发到MQTT
- 任务进度通过`mavlink_mission`节点监听MAVLink消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用

## 使用示例

### 通过MQTT发送任务

使用MQTT客户端（如`mosquitto_pub`）发送任务：

```bash
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavmission/waypoints/uav1" \
  -m '{
    "waypoints": [
      [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
      [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
    ],
    "auto_trigger": true
  }'
```

### 订阅任务状态

```bash
mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavmission/status/uav1"
```

## ROS2话题

**发布的话题：**
- `/mission/waypoints` (std_msgs/Float32MultiArray): 航点数据，发送给`mavlink_mission`节点上传到PX4
  - QoS: `Reliable`（匹配`mavlink_mission`的订阅QoS）
- `/px4_mission/trigger` (std_msgs/Bool): 任务触发信号，发送给`px4_mission`节点执行任务
  - QoS: `BestEffort`（匹配`px4_mission`的订阅QoS）

**订阅的话题：**
- `/px4_mission/state` (std_msgs/String): 任务状态（JSON格式），从`px4_mission`节点订阅并转发到MQTT
  - QoS: `BestEffort`（匹配`px4_mission`的发布QoS）
  - 更新频率：2Hz（由`px4_mission`节点控制）

## 注意事项

1. **任务系统依赖**: `mqtt_mission`需要`px4_control`包中的`px4_mission`节点和`mavlink_mission`节点运行
2. **架构一致性**: `mqtt_mission`的架构与`mqtt_estimator`和`mqtt_control`一致，都通过中间节点与PX4通信
3. **航点格式**: 每个航点必须包含9个参数：`[frame, command, lat, lon, alt, param1, param2, param3, param4]`
4. **坐标系**: 推荐使用`frame=3`（全局相对高度，FRAME_GLOBAL_REL_ALT）
5. **MAVLink协议**: 航点通过MAVLink Mission协议上传到PX4的`dataman`存储，确保任务持久化
6. **自动触发**: 如果`auto_trigger=true`，任务会在上传后1秒自动触发执行
7. **状态更新**: 任务状态实时从`px4_mission`节点获取并转发到MQTT，更新频率为2Hz
8. **任务进度跟踪**: 任务进度通过`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，不依赖`mission_result` uORB话题
9. **QoS配置**: 
   - `/mission/waypoints`使用`Reliable` QoS确保航点数据可靠传输
   - `/px4_mission/trigger`使用`BestEffort` QoS匹配`px4_mission`的订阅
   - `/px4_mission/state`使用`BestEffort` QoS匹配`px4_mission`的发布
   _ `/
10. **GPS要求**: 发送任务前必须确保GPS可用且已设置home位置
11. **错误处理**: 如果命令格式错误或缺少必需参数，节点会记录警告日志但不会崩溃

## 文件结构

```
px4_mqtt/
├── config/
│   ├── mqtt_estimator.yaml          # Estimator配置文件
│   └── mqtt_control.yaml             # Control配置文件
├── include/px4_mqtt/
│   ├── mqtt_estimator.hpp           # Estimator头文件
│   ├── mqtt_control.hpp             # Control头文件
│   └── mqtt_mission.hpp             # Mission头文件
├── src/
│   ├── mqtt_estimator.cpp           # Estimator源文件
│   ├── mqtt_control.cpp             # Control源文件
│   └── mqtt_mission.cpp             # Mission源文件
├── launch/
│   ├── mqtt_estimator.launch.py     # Estimator Launch文件
│   ├── mqtt_control.launch.py       # Control Launch文件
│   └── mqtt_mission.launch.py       # Mission Launch文件
├── config/
│   ├── mqtt_estimator.yaml          # Estimator配置文件
│   ├── mqtt_control.yaml            # Control配置文件
│   └── mqtt_mission.yaml            # Mission配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 示例：完整启动流程

```bash
# 1. 编译
cd ~/px4_sitl/px4_ros2_ws
colcon build --packages-select px4_mqtt px4_control
source install/setup.bash

# 2. 编辑配置文件（可选）
# 编辑 config/mqtt_estimator.yaml

# 3. 启动px4_estimator
ros2 launch px4_control px4_estimator.launch.py

# 4. 启动mqtt_estimator（新终端）
ros2 launch px4_mqtt mqtt_estimator.launch.py

# 5. 验证数据流
# 终端3：检查ROS2话题
ros2 topic echo /px4_estimator/state

# 终端4：使用MQTT客户端订阅（需要安装mosquitto-clients）
mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. -t "uavcontrol/state/uav1"
```

## 许可证

BSD 3-Clause License

---

# MQTT Servo 节点（mqtt_servo_control）

新增的 `mqtt_servo_control` 节点用于通过 MQTT 接收舵机（servo）命令，并在 ROS2 中发布 `px4_msgs::msg::ServoCommand` 消息，供 `px4_control` 或其他订阅者转发到 PX4 或本地执行。

主要用途：
- 通过 MQTT 下发逐通道舵机/伺服输出值（普通六通道示例），便于地面站或远端控制器直接控制单个舵机或一组舵机。
- 在开发与测试阶段快速验证舵机控制逻辑，或作为 vehicle_command/drivers 的替代/补充路径。

注意：该节点只是把解析后的命令发布为 `px4_msgs::msg::ServoCommand`，是否最终作用到物理舵机取决于 `px4_control` 中的实现（`PX4ServoController`）和 PX4 的配置（例如是否支持 MAV_CMD_DO_SET_SERVO，或是否需要将命令映射到 actuator_controls）。有关后续改进请参见 TODO 列表中的 actuator_controls 后备实现项目。

配置文件
---------
配置文件放在 `src/px4_mqtt/config/mqtt_servo.yaml`（开发时会优先读取源目录，见上文“配置”一节）。该文件包含以下参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID（用于拼接 MQTT topic 和 client id） | `uav1` |
| `mqtt_broker` | MQTT 服务器地址（支持 `tcp://host:port` 或 `mqtt://host:port`，节点会把 `mqtt://` 转为 `tcp://`） | `tcp://118.195.156.74:5704` |
| `mqtt_username` | MQTT 用户名（可空） | `uav1` |
| `mqtt_password` | MQTT 密码（可空） | `Aa123456.` |
| `mqtt_servo_topic` | MQTT 舵机命令主题前缀（最终主题为 `{mqtt_servo_topic}{uav_name}`） | `uavcontrol/servo/` |

示例（`src/px4_mqtt/config/mqtt_servo.yaml`）：

```yaml
mqtt_servo:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_servo_topic: "uavcontrol/servo/"
```

消息格式（MQTT payload）
-------------------------
节点期望接收到 JSON 格式的舵机命令。推荐的字段如下：

示例（6 通道）：

```json
{
  "actuators": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
  "actuator_index": 0,
  "send_frequency": 10.0,
  "update": true
}
```

字段说明：
- `actuators`: 舵机输出值数组（float），长度根据实现可为固定 6 个值或更多；数值范围/含义由上层协议或 `px4_control` 决定（通常为归一化值或 PWM）。
- `actuator_index`: 起始通道索引（uint8），用于指定哪一路舵机通道开始应用数组值。
- `send_frequency`: 发送频率，用于告诉接收端以何频率重复/维持该命令（Hz）。
- `update`: 是否立即生效（bool），含义由接收端解释。

运行与测试
-----------

1) 使用参数文件启动（推荐）

```bash
# 启动（示例）
source install/setup.bash
ros2 run px4_mqtt mqtt_servo_control --ros-args --params-file src/px4_mqtt/config/mqtt_servo.yaml
```

或在 launch 中加入参数文件：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

config_dir = Path(__file__).resolve().parents[1] / 'config'
servo_params = str(config_dir / 'mqtt_servo.yaml')

node = Node(
    package='px4_mqtt',
    executable='mqtt_servo_control',
    name='mqtt_servo_control',
    parameters=[servo_params]
)

ld = LaunchDescription([node])

```

2) 发送测试 MQTT 消息（mosquitto_pub）

```bash
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/servo/uav1" \
  -m '{"actuators":[0.1,0.2,0.3,0,0,0],"actuator_index":0,"send_frequency":10.0,"update":true}'
```

3) 在 ROS 端查看发布的 `ServoCommand`（如果节点工作正常，会发布到 `/px4_servo/command`）：

```bash
ros2 topic echo /px4_servo/command
```

如果需要只查看一条消息（某些 ros2 版本不支持 `-n`），可以使用 `timeout`：

```bash
timeout 5s ros2 topic echo /px4_servo/command | sed -n '1,200p'
```

4) Python 发布示例（使用 paho-mqtt）

```python
import paho.mqtt.publish as publish
import json

payload = {
    "actuators": [0.1,0.2,0.3,0,0,0],
    "actuator_index": 0,
    "send_frequency": 10.0,
    "update": True
}

publish.single("uavcontrol/servo/uav1", json.dumps(payload), hostname="118.195.156.74", port=5704, auth={"username":"uav1","password":"Aa123456."})
```

常见问题与排查建议
--------------------
- YAML 解析失败：如果通过 `--params-file` 启动时报错 `Failed to parse global arguments` 或类似的 RCLInvalidROSArgsError，请检查 YAML 文件是否包含制表符（tab）而非空格，或有非法字符。用 `cat -A` 或 `hexdump -C` 检查不可见字符。
- 未生成 `/px4_servo/command`：确认 `mqtt_servo_control` 已成功连接到 MQTT（查看节点日志），并确保发布到的 MQTT topic 为 `{mqtt_servo_topic}{uav_name}`（例如 `uavcontrol/servo/uav1`）。
- 无法连接到 MQTT：检查 broker 地址/端口、用户名/密码、网络连通性（ping/telnet）以及 broker 日志。
- ROS2 参数未加载：启动时可把 `--params-file` 路径写成绝对路径，或在 launch 中以 `parameters=[str(path)]` 形式传入。

与 PX4 的集成注意事项
---------------------
- `mqtt_servo_control` 只负责在 ROS2 中发布 `px4_msgs::msg::ServoCommand`。如何把该消息最终转发到 PX4（例如通过 `px4_control` 节点触发 MAV_CMD_DO_SET_SERVO，或映射到 `actuator_controls`）取决于 `px4_control` 的实现与 PX4 的运行时支持。请查看 `px4_control` 中的 `PX4ServoController` 实现并根据需要启用 actuator_controls 后备实现（见 TODO）。
- 如果目标是直接用 MAV_CMD_DO_SET_SERVO，请确认 PX4 支持该命令并在 commander 中不会被拒绝（某些 PX4 配置或模式下会忽略来自 ROS 的命令）。

开发者/维护说明
-----------------
- 配置文件路径：源目录 `src/px4_mqtt/config/mqtt_servo.yaml`（开发优先使用），安装后会被复制到 `install/px4_mqtt/share/px4_mqtt/config/mqtt_servo.yaml`。
- 依赖：和其它 mqtt 节点一致，需要系统库 `libpaho-mqttcpp-dev` 与 `nlohmann-json3-dev`，以及 ROS2 依赖 `rclcpp`、`px4_msgs` 等。
- 日志：节点启动后的 stdout/stderr 会记录连接与解析错误；在使用 launch 时可通过 `ros2 launch` 的日志输出或 `ros2 run` 重定向到文件查看。

下一步建议（可选）
--------------------
1. 若要保证命令能可靠执行在 PX4，建议实现 actuator_controls 后备路径（将命令映射到 `/fmu/out/actuator_controls_*`），并在 README 中说明风险与映射规则。该项已列入 TODO 列表。  
2. 添加一个简单的 `launch/mqtt_servo.launch.py`，把 `mqtt_servo.yaml` 自动加载并提供可替换的 launch 参数。  
3. 编写一个小型集成测试（Python），在 CI 或本地验证从 MQTT 到 ROS 到 PX4 的端到端流。
