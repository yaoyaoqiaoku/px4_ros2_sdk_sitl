# PX4 Control Package

This package contains ROS2 nodes for monitoring and controlling PX4 autopilot.

（中文注释）
本包包含用于监控和控制 PX4 飞控的多个 ROS2 节点，提供状态监控、手柄桥接、任务规划与飞行控制接口。

---

# PX4 Control 包（中文注释在英文段落后）

本文件采用：先英文原文，再在每个小节后附上中文注释，方便阅读和团队共享。

## Nodes

### px4_estimator

A comprehensive status monitoring node that subscribes to multiple PX4 topics and displays the information in a formatted terminal output.

（中文注释）
px4_estimator：一个综合状态监控节点，订阅 PX4 的多个话题并以格式化的终端输出展示信息。

#### Subscribed Topics

**Sensor Related:**
- `/fmu/out/sensor_combined` - IMU data (gyroscope and accelerometer)
- `/fmu/out/vehicle_gps_position` - GPS position data
- `/fmu/out/timesync_status` - Time synchronization status

（中文注释）
传感器相关：
- `/fmu/out/sensor_combined` - IMU 数据（陀螺仪和加速度计）
- `/fmu/out/vehicle_gps_position` - GPS 位置数据
- `/fmu/out/timesync_status` - 时间同步状态

**Vehicle State:**
- `/fmu/out/vehicle_attitude` - Vehicle attitude (quaternion)
- `/fmu/out/vehicle_local_position` - Local position (NED frame)
- `/fmu/out/vehicle_global_position` - Global position (lat/lon/alt)
- `/fmu/out/vehicle_odometry` - Odometry data

（中文注释）
车辆状态：
- `/fmu/out/vehicle_attitude` - 车辆姿态（四元数）
- `/fmu/out/vehicle_local_position` - 本地位置（NED 坐标系）
- `/fmu/out/vehicle_global_position` - 全局位置（经度/纬度/高度）
- `/fmu/out/vehicle_odometry` - 里程计数据

**Control Related:**
- `/fmu/out/vehicle_control_mode` - Current control mode
- `/fmu/out/manual_control_setpoint` - Manual control inputs
- `/fmu/out/position_setpoint_triplet` - Position setpoints

（中文注释）
控制相关：
- `/fmu/out/vehicle_control_mode` - 当前控制模式
- `/fmu/out/manual_control_setpoint` - 人工控制输入
- `/fmu/out/position_setpoint_triplet` - 位置设定点

**Health Status:**
- `/fmu/out/vehicle_status` - Vehicle status (armed, nav_state, etc.)
- `/fmu/out/estimator_status_flags` - EKF estimator status
- `/fmu/out/failsafe_flags` - Failsafe flags
- `/fmu/out/battery_status` - Battery status
- `/fmu/out/vehicle_command_ack` - Command acknowledgments

（中文注释）
健康/状态信息：
- `/fmu/out/vehicle_status` - 车辆状态（armed 等）
- `/fmu/out/estimator_status_flags` - EKF 状态标志
- `/fmu/out/failsafe_flags` - 故障保护标志
- `/fmu/out/battery_status` - 电池状态
- `/fmu/out/vehicle_command_ack` - 命令确认

#### Usage

**Method 1: Using launch file**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_estimator.launch.py
```

（中文注释）
使用方法1：通过 launch 文件启动（推荐）

**Method 2: Direct execution**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 run px4_control px4_estimator
```

（中文注释）
方法2：直接运行可执行文件，适合调试。

The node will display a real-time status dashboard that updates every 500ms, showing:
- Sensor data (IMU, GPS, time sync)
- Vehicle state (attitude, position, velocity)
- Control information (control modes, manual inputs, setpoints)
- Health status (vehicle status, estimator flags, failsafe, battery)

（中文注释）
该节点会以仪表盘形式每 500ms 更新，显示：传感器数据、车辆状态、控制信息、健康状态等。

#### DDS QoS Configuration

The node uses optimized DDS QoS profiles according to PX4 uXRCE-DDS documentation:

- **Sensor Data QoS** (sensor_combined, vehicle_gps_position):
  - Reliability: BEST_EFFORT (loss-tolerant, high-frequency data)
  - History: KEEP_LAST with depth 5

（中文注释）
QoS 建议（传感器类）：
- 可靠性：BEST_EFFORT（高频允许丢包）
- 历史：KEEP_LAST，深度 5

- **Status/Control QoS** (vehicle_status, vehicle_control_mode, failsafe_flags, etc.):
  - Reliability: BEST_EFFORT
  - Durability: TRANSIENT_LOCAL (messages persist for late-joining subscribers)
  - History: KEEP_LAST with depth 10

（中文注释）
状态/控制类 QoS 建议：
- 可靠性：BEST_EFFORT
- 持久性：TRANSIENT_LOCAL（允许后加入订阅者获取最近消息）
- 历史：KEEP_LAST，深度 10

- **Default QoS** (vehicle_attitude, vehicle_local_position, etc.):
  - Reliability: BEST_EFFORT
  - History: KEEP_LAST with depth 10

（中文注释）
默认 QoS：BEST_EFFORT / KEEP_LAST（深度 10）。

Press Ctrl+C to exit.

（中文注释）
按 Ctrl+C 退出。

---

### px4_control

A control node that provides an interface for controlling PX4 vehicles, including:
- **Arming/Disarming**: Unlock and lock the vehicle motors
- **Mode Switching**: Switch between different flight modes (Offboard, Auto Takeoff, Auto Land, Auto Loiter/Hold)
- **Offboard Control**: Position and velocity control in Offboard mode

（中文注释）
px4_control：控制节点，提供解锁/上锁、模式切换、Offboard 控制（位置/速度）等功能。

#### Published Topics

- `/fmu/in/vehicle_command` - Vehicle commands (ARM, DISARM, mode switching, takeoff, land)
- `/fmu/in/offboard_control_mode` - Offboard control mode (must be published before entering Offboard mode)
- `/fmu/in/trajectory_setpoint` - Trajectory setpoints for position/velocity control in Offboard mode

（中文注释）
发布话题：
- `/fmu/in/vehicle_command` - 车辆命令
- `/fmu/in/offboard_control_mode` - Offboard 控制模式（进入前需发布）
- `/fmu/in/trajectory_setpoint` - 轨迹/设定点

#### Subscribed Topics

- `/fmu/out/vehicle_status` - Vehicle status (armed state, navigation state)
- `/fmu/out/vehicle_command_ack` - Command acknowledgments
- `/fmu/out/vehicle_local_position` - Local position for maintaining altitude during velocity control

（中文注释）
订阅话题：
- `/fmu/out/vehicle_status` - 车辆状态
- `/fmu/out/vehicle_command_ack` - 命令确认
- `/fmu/out/vehicle_local_position` - 本地位置（用于速度控制维持高度）

#### Public API

**Arming/Disarming:**
```cpp
void arm();           // Arm (unlock) the vehicle
void disarm();         // Disarm (lock) the vehicle
```

（中文注释）
接口示例（上锁/解锁）。

**Mode Switching:**
```cpp
void set_mode_offboard();     // Switch to Offboard mode
void set_mode_land();         // Switch to AUTO.LAND mode
void set_mode_hold();         // Switch to AUTO.LOITER (HOLD) mode
```

（中文注释）
模式切换接口示例。

**Takeoff:**
```cpp
void arm_and_takeoff();  // Arm then switch to AUTO.TAKEOFF mode
```

（中文注释）
起飞示例接口。

**Offboard Control:**
```cpp
// Position control (NED frame: z is negative for altitude)
void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);

// Velocity control (NED frame)
void publish_velocity_setpoint(float vx, float vy, float vz, float yaw = 0.0f);

// Advanced: Custom offboard control mode and trajectory setpoint
void publish_offboard_control_mode(bool position, bool velocity, bool acceleration, 
                                    bool attitude, bool body_rate);
void publish_trajectory_setpoint(float x, float y, float z, 
                                 float vx, float vy, float vz,
                                 float yaw, float yawspeed);
```

（中文注释）
Offboard 控制接口示例（位置/速度/轨迹）。

#### Usage

**Method 1: Using launch file**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_control.launch.py
```

（中文注释）
使用 launch 文件启动 px4_control

**Method 2: Direct execution**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 run px4_control px4_control
```

（中文注释）
直接运行可执行文件。

**Method 3: Using CLI script**
```bash
# Arm and takeoff
python3 src/px4_control/scripts/px4_control_cli.py arm_and_takeoff

# Switch to offboard mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_offboard

# Send position setpoint (hover at 5m altitude)
python3 src/px4_control/scripts/px4_control_cli.py position 0 0 -5 0

# Send velocity setpoint (move forward at 1 m/s)
python3 src/px4_control/scripts/px4_control_cli.py velocity 1 0 0 0

# Switch to land mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_land

# Switch to hold mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_hold
```

（中文注释）
CLI 示例：基于脚本快速发送命令。

#### Important Notes

1. **Offboard Mode Requirements**: 
   - Before entering Offboard mode, you must publish `/fmu/in/offboard_control_mode` at least at 2Hz
   - The node automatically maintains this publication when `offboard_mode_active_` is true
   - After switching to Offboard mode, wait a few seconds before sending setpoints

（中文注释）
重要说明（Offboard）：进入 Offboard 前需保持 `/fmu/in/offboard_control_mode` 的发布频率 >= 2Hz，且进入后持续发送 setpoint。

2. **Coordinate Frame**: 
   - All positions and velocities use NED (North-East-Down) frame
   - Z-axis is negative for altitude (e.g., -5.0 means 5 meters above ground)
   - Yaw angle is in radians, range [-PI, PI]

（中文注释）
坐标系：NED，Z 轴负值表示高度，偏航以弧度表示。

3. **Arming Requirements**:
   - Vehicle must be in a safe state (not in failsafe)
   - Pre-flight checks must pass
   - For Offboard mode, vehicle must be armed before entering

（中文注释）
上锁条件：确保车辆处于安全状态且预检通过，Offboard 需先上锁。

4. **Mode Switching**:
   - Mode changes may take a few seconds to take effect
   - Check `/fmu/out/vehicle_status` to verify mode changes

（中文注释）
模式切换可能需要几秒，检查 `/fmu/out/vehicle_status` 验证。

---

### joystick_bridge

A node that reads input from USB joystick/gamepad devices and publishes to ROS2 `/joy` topic.

（中文注释）
joystick_bridge：读取本机 USB 手柄并发布 `/joy` 话题。

#### Published Topics

- `/joy` (sensor_msgs/msg/Joy) - Joystick data with axes and buttons

（中文注释）
发布 `/joy`。

#### Usage

**Using launch file (recommended):**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_joystick.launch.py
```

（中文注释）
通过 launch 启动手柄桥接。

**Direct execution:**
```bash
ros2 run px4_control joystick_bridge
```

（中文注释）
直接运行示例。

**Parameters:**
- `device`: Joystick device path (default: `/dev/input/js0`)
- `publish_rate`: Publishing rate in Hz (default: `50.0`)

（中文注释）
参数说明：设备路径与发布频率。

#### Joystick Control Setup

**1. Check joystick device:**
```bash
ls -la /dev/input/js*
```

（中文注释）
检查是否存在 joystick 设备。

**2. Set device permissions (if needed):**
```bash
sudo chmod 666 /dev/input/js0
# Or add user to input group:
sudo usermod -a -G input $USER
```

（中文注释）
设置设备权限或将用户加入 input 组。

**3. Configure PX4:**
- Set `COM_RC_IN_MODE = 1` (Joystick) in QGroundControl or via MAVLink
- Ensure PX4 is in a manual control mode (Manual/Stabilized/Position/Altitude)

（中文注释）
在 QGroundControl 中设置 `COM_RC_IN_MODE = 1` 以使用 ROS2 手柄输入。

**Important Note about QGroundControl Joystick:**
QGroundControl has its own joystick driver that directly reads from `/dev/input/js*` devices, independent of ROS2. This means:
- QGC's joystick calibration page shows QGC's own joystick readings, not ROS2 data
- Both QGC and ROS2 can access the joystick simultaneously
- To use only ROS2 joystick control:
  1. In QGC, go to **Vehicle Setup > Joystick**
  2. Disconnect/disable the joystick in QGC settings
  3. Or simply don't configure joystick in QGC - PX4 will use ROS2 input when `COM_RC_IN_MODE = 1`
- The ROS2 joystick data is sent via `/fmu/in/manual_control_input` topic with `data_source = SOURCE_MAVLINK_0`
- PX4's `ManualControlSelector` will prioritize the ROS2 input when properly configured

（中文注释）
关于 QGC 的注意：QGC 有独立驱动，若需只使用 ROS2 输入，请在 QGC 中禁用 joystick 或不配置。

**4. Verify data flow:**
```bash
# Check joystick data
ros2 topic echo /joy

# Check PX4 input
ros2 topic echo /fmu/in/manual_control_input

# Check PX4 output
ros2 topic echo /fmu/out/manual_control_setpoint
```

（中文注释）
验证数据流：查看 `/joy`、`/fmu/in/manual_control_input` 与 `/fmu/out/manual_control_setpoint`。

**5. Customize axis mapping:**
```bash
ros2 launch px4_control px4_joystick.launch.py \
    roll_axis:=0 \
    pitch_axis:=1 \
    throttle_axis:=2 \
    yaw_axis:=3 \
    roll_inverted:=false \
    dead_zone:=0.05
```

（中文注释）
自定义轴映射示例。

#### Troubleshooting

- **No joystick detected**: Check `lsusb` and `/dev/input/js*`
- **Permission denied**: Run `sudo chmod 666 /dev/input/js0` or add user to input group
- **PX4 not responding**: 
  - Verify `COM_RC_IN_MODE = 1`
  - Check that `/fmu/in/manual_control_input` has data
  - Ensure PX4 is armed and in correct flight mode

（中文注释）
常见问题与解决方法。

---

### px4_joystick

A converter node that subscribes to `/joy` topic and converts joystick input to PX4 `manual_control_input` messages for controlling the vehicle.

（中文注释）
px4_joystick：把 `/joy` 转换为 `manual_control_input` 并发送给 PX4。

#### Subscribed Topics

- `/joy` (sensor_msgs/msg/Joy) - Joystick input data

（中文注释）
订阅 `/joy`。

#### Published Topics

- `/fmu/in/manual_control_input` (px4_msgs/msg/ManualControlSetpoint) - Manual control commands sent to PX4

（中文注释）
发布 `/fmu/in/manual_control_input` 给 PX4。

#### Usage

**Using launch file (recommended):**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_joystick.launch.py
```

（中文注释）
使用 launch 启动 px4_joystick。

**Direct execution:**
```bash
ros2 run px4_control px4_joystick
```

（中文注释）
直接运行示例。

**Parameters:**
- `roll_axis`: Joystick axis index for roll control (default: `0`)
- `pitch_axis`: Joystick axis index for pitch control (default: `1`)
- `throttle_axis`: Joystick axis index for throttle control (default: `2`)
- `yaw_axis`: Joystick axis index for yaw control (default: `3`)
- `roll_inverted`: Invert roll axis (default: `false`)
- `pitch_inverted`: Invert pitch axis (default: `false`)
- `throttle_inverted`: Invert throttle axis (default: `false`)
- `yaw_inverted`: Invert yaw axis (default: `false`)
- `dead_zone`: Dead zone threshold (default: `0.05`)

（中文注释）
参数说明。

---

### One-Click Startup Script

A convenient script to launch all required components for PX4 SITL simulation with ROS2 control.

（中文注释）
一键启动脚本：用于在本地启动 Gazebo、PX4 SITL、MicroXRCEAgent、以及本包的节点。

#### Usage

```bash
cd ~/px4_sitl/px4_ros2_ws
./src/px4_control/sh/px4_control.sh
```

（中文注释）
用法示例。

This script will automatically launch:
1. **Gazebo Simulator** - Starts the Gazebo simulation environment
2. **PX4 SITL** - Compiles and launches PX4 Software-In-The-Loop
3. **MicroXRCEAgent** - DDS agent for communication between PX4 and ROS2
4. **px4_estimator** - Status monitoring node
5. **px4_control** - Control node for vehicle commands
6. **px4_joystick** - Joystick bridge and converter nodes

（中文注释）
脚本启动的组件说明。

**Note**: Make sure you have sourced the ROS2 workspace before running the script:
```bash
source install/setup.bash
```

（中文注释）
提醒：运行前请 source 工作区。

---

### Mission Planning System

A waypoint mission planning system that allows you to upload and execute waypoint missions on PX4.

（中文注释）
任务规划系统：支持上传与执行航点任务。

#### Architecture Overview

The mission planning system uses a three-component architecture:

```
┌─────────────────────┐      ┌──────────────────┐
│  px4_mission_cli.py  │      │   mqtt_mission   │  MQTT bridge
│   (Python Script)   │      │   (C++ Node)     │
└──────────┬──────────┘      └────────┬─────────┘
           │                          │
           ├─→ /mission/waypoints ───┼──→ (Float32MultiArray)
           │         ↓                │
           │  ┌──────────────────────┐ │
           │  │  mavlink_mission.py  │ │  MAVLink uploader
           │  │    (ROS2 Node)       │ │
           │  └──────────┬───────────┘ │
           │             │             │
           │             └─→ MAVLink Protocol → PX4 dataman
           │
           ├─→ /mission/trigger (Bool) ───┐
           │                                │
           └─→ /px4_mission/trigger ────────┼──→ (Bool)
                                            ↓
                                    ┌──────────────────┐
                                    │   px4_mission    │  Mission executor & monitor
                                    │   (C++ Node)     │
                                    └────────┬─────────┘
                                             │
                                             ├─→ /fmu/in/vehicle_command → PX4 execution
                                             │
                                             └─→ /px4_mission/state (JSON) → mqtt_mission → MQTT
```

（中文注释）
任务规划系统采用三个组件的架构：CLI 工具、MAVLink 上传器、任务执行器。

#### Component Details

**1. px4_mission_cli.py** (CLI Tool)
- **Location**: `scripts/px4_mission_cli.py`
- **Purpose**: Command-line interface for sending waypoint missions
- **Input**: Command-line arguments (waypoints string or JSON file)
- **Output**: 
  - Publishes waypoints to `/mission/waypoints` topic
  - Publishes trigger signal to `/mission/trigger` topic
- **Features**:
  - Parse waypoints from command line string or JSON file
  - Validate waypoint format (9 parameters per waypoint)
  - Optional mission trigger (use `--no-trigger` to upload only)

（中文注释）
组件详情：
1. px4_mission_cli.py：命令行工具，用于发送航点任务。

**2. mavlink_mission.py** (MAVLink Uploader)
- **Location**: `scripts/mavlink_mission.py`
- **Purpose**: Receive waypoints via ROS2 and upload to PX4 via MAVLink Mission Protocol
- **Subscribed Topics**:
  - `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data
- **MAVLink Connection**: 
  - Default: `udp:127.0.0.1:14540` (configurable via ROS2 parameter)
  - Uses `pymavlink` library for MAVLink communication
- **Protocol Flow**:
  1. Clear existing mission (`MISSION_CLEAR_ALL`)
  2. Send mission count (`MISSION_COUNT`)
  3. Receive mission request (`MISSION_REQUEST`)
  4. Send waypoints one by one (`MISSION_ITEM`)
  5. Receive final acknowledgment (`MISSION_ACK`)
- **Thread Safety**: Uses separate thread for upload to avoid blocking ROS2 callbacks

（中文注释）
2. mavlink_mission.py：接收 ROS2 航点并通过 MAVLink 协议上传至 PX4。

**3. px4_mission** (Mission Executor)
- **Location**: 
  - Header: `include/px4_mission/px4_mission.hpp`
  - Implementation: `utils/px4_mission.cpp`
  - Node wrapper: `utils/px4_mission_node.cpp`
- **Purpose**: Execute missions and monitor mission status
- **Subscribed Topics**:
  - `/px4_mission/trigger` (std_msgs/Bool): Mission trigger signal (from mqtt_mission or other nodes)
  - `/mission/trigger` (std_msgs/Bool): Legacy mission trigger signal (backward compatibility)
  - `/mission/count` (std_msgs/UInt16): Total waypoint count from mavlink_mission
  - `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index from mavlink_mission
  - `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus): Vehicle state
  - `/fmu/out/vehicle_global_position` (px4_msgs/VehicleGlobalPosition): GPS position
  - `/fmu/out/vehicle_command_ack` (px4_msgs/VehicleCommandAck): Command acknowledgments
- **Published Topics**:
  - `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand): Vehicle commands
  - `/px4_mission/state` (std_msgs/String): Mission state in JSON format (for mqtt_mission bridge)
- **State Machine**:
  ```
  IDLE → ARMING → MISSION_ACTIVE → MISSION_COMPLETE → IDLE
  ```
- **Execution Flow**:
  1. **IDLE**: Wait for mission trigger (status not displayed)
  2. **ARMING**: 
     - Wait for mission availability (checks mission_count or 3s timeout for MAVLink upload)
     - Set mode to AUTO_MISSION (nav_state=3)
     - Arm vehicle
     - Start mission execution
  3. **MISSION_ACTIVE**: 
     - Monitor mission progress via `/mission/count` and `/mission/current_waypoint` topics
     - Display status: State, Nav mode, Armed status, Waypoint progress
     - Detect completion (current_waypoint >= total_waypoints or nav_state changes to 4=HOLD, 5=RTL, or 6=LAND)
  4. **MISSION_COMPLETE**: 
     - Mission completed, return to IDLE
     - No automatic RTL (user can set RTL waypoint as last waypoint in mission)
  5. **IDLE**: Reset and wait for next mission

（中文注释）
3. px4_mission：任务执行器，负责执行任务与状态监控。

#### File Structure

```
px4_control/
├── scripts/
│   ├── px4_mission_cli.py      # CLI tool for sending missions
│   └── mavlink_mission.py       # MAVLink mission uploader
├── include/
│   └── px4_mission/
│       └── px4_mission.hpp      # Mission executor header
├── utils/
│   ├── px4_mission.cpp          # Mission executor implementation
│   └── px4_mission_node.cpp     # ROS2 node wrapper
└── launch/
    └── px4_mission.launch.py    # Launch file for mission system
```

（中文注释）
文件结构说明。

#### Installation

Install required dependencies:
```bash
pip3 install pymavlink
```

（中文注释）
安装依赖。

#### Usage

**Step 1: Start the mission system**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_mission.launch.py
```

This launches:
- `mavlink_mission.py`: Waits for waypoints on `/mission/waypoints`, uploads via MAVLink, publishes `/mission/count` and `/mission/current_waypoint`
- `px4_mission`: Waits for trigger on `/px4_mission/trigger` (or `/mission/trigger` for backward compatibility), publishes `/px4_mission/state`

（中文注释）
第一步：启动任务系统。

**Step 2: Send a mission**

```bash
# Using JSON file
ros2 run px4_control px4_mission_cli.py --file examples/example_mission.json

# Using command line waypoints (format: "wp1|wp2|wp3|...")
ros2 run px4_control px4_mission_cli.py --waypoints \
  "3,16,47.397742,8.545594,5.0,0.0,1.0,0.0,0.0|3,16,47.397842,8.545594,5.0,0.0,1.0,0.0,0.0"

# Upload only (don't trigger execution)
ros2 run px4_control px4_mission_cli.py --file examples/example_mission.json --no-trigger
```

**Alternative: Using MQTT (requires `mqtt_mission` node)**

```bash
# Start mqtt_mission node
ros2 launch px4_mqtt mqtt_mission.launch.py

# Send mission via MQTT (using mosquitto_pub)
mosquitto_pub -h <broker> -p <port> -u <username> -P <password> \
  -t "uavmission/waypoints/uav1" \
  -m '{"waypoints": [[3,16,47.397742,8.545594,5.0,0.0,1.0,0.0,0.0]], "auto_trigger": true}'
```

（中文注释）
第二步：发送任务。

**Step 3: Monitor mission execution**

The `px4_mission` node will:
- Display status updates every 2 seconds (only when mission is active)
- Show current state, navigation mode, armed status, waypoint progress
- Status display is suppressed in IDLE state to reduce clutter
- Mission completion returns to IDLE (no automatic RTL - set RTL waypoint as last waypoint if needed)

（中文注释）
第三步：监控任务执行。

#### Waypoint Format

Each waypoint consists of 9 parameters:
```
[frame, command, lat, lon, alt, param1, param2, param3, param4]
```

- **frame**: Mission frame (3 = FRAME_GLOBAL_REL_ALT)
- **command**: MAVLink command (16 = NAV_WAYPOINT)
- **lat, lon**: Latitude and longitude in degrees
- **alt**: Altitude in meters (relative to home)
- **param1**: Hold time in seconds
- **param2**: Acceptance radius in meters
- **param3**: Pass radius in meters (0 = pass through)
- **param4**: Yaw angle in radians (0 = no change)

（中文注释）
航点格式说明。

#### JSON Mission File Format

```json
{
  "waypoints": [
    {
      "frame": 3,
      "command": 16,
      "lat": 47.397742,
      "lon": 8.545594,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": 0.0
    }
  ]
}
```

（中文注释）
JSON 文件格式示例。

#### Execution Flow Details

**Mission Upload Flow:**
1. User runs `px4_mission_cli.py` with waypoints
2. CLI parses and validates waypoints
3. CLI publishes waypoints to `/mission/waypoints`
4. `mavlink_mission.py` receives waypoints
5. MAVLink uploader connects to PX4 via UDP
6. Clears existing mission
7. Uploads waypoints via MAVLink Mission Protocol
8. Stores mission in PX4 dataman

**Mission Execution Flow:**
1. CLI publishes trigger (`true`) to `/mission/trigger`
2. `px4_mission` receives trigger, transitions to ARMING state
3. Waits for mission availability (checks `/mission/count` topic or 3s timeout for MAVLink upload)
4. Sets mode to AUTO_MISSION (nav_state=3)
5. Arms vehicle if not already armed
6. Sends `VEHICLE_CMD_MISSION_START`
7. Transitions to MISSION_ACTIVE state
8. Monitors mission progress via `/mission/count` and `/mission/current_waypoint` topics from mavlink_mission
9. Detects completion when:
   - `current_waypoint >= total_waypoints`, OR
   - `nav_state` changes from 3 (AUTO_MISSION) to 4 (HOLD), 5 (RTL), or 6 (LAND)
10. Transitions to MISSION_COMPLETE state
11. Returns to IDLE state (no automatic RTL - user can set RTL waypoint as last waypoint)
12. Publishes mission state to `/px4_mission/state` topic (JSON format) for MQTT bridge

（中文注释）
任务上传与执行流程详解。

#### ROS2 Topics

**Published Topics:**
- `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data from CLI or mqtt_mission
- `/mission/trigger` (std_msgs/Bool): Mission trigger signal from CLI (legacy, backward compatibility)
- `/px4_mission/trigger` (std_msgs/Bool): Mission trigger signal from mqtt_mission or other nodes
- `/mission/count` (std_msgs/UInt16): Total waypoint count from mavlink_mission
- `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index from mavlink_mission
- `/px4_mission/state` (std_msgs/String): Mission state in JSON format (for mqtt_mission bridge)
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand): Vehicle commands from px4_mission

**Subscribed Topics:**
- `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data (subscribed by mavlink_mission)
- `/px4_mission/trigger` (std_msgs/Bool): Mission trigger (subscribed by px4_mission, primary)
- `/mission/trigger` (std_msgs/Bool): Mission trigger (subscribed by px4_mission, legacy for backward compatibility)
- `/mission/count` (std_msgs/UInt16): Total waypoint count (subscribed by px4_mission)
- `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index (subscribed by px4_mission)
- `/px4_mission/state` (std_msgs/String): Mission state (subscribed by mqtt_mission)
- `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus): Vehicle state (subscribed by px4_mission)
- `/fmu/out/vehicle_global_position` (px4_msgs/VehicleGlobalPosition): GPS position (subscribed by px4_mission)
- `/fmu/out/vehicle_command_ack` (px4_msgs/VehicleCommandAck): Command acknowledgments (subscribed by px4_mission)

（中文注释）
ROS2 话题说明。

#### Important Notes

- **MAVLink Protocol**: Waypoints are uploaded via MAVLink Mission Protocol to PX4's dataman storage
- **ROS2 Commands**: Mission execution is controlled via ROS2 VehicleCommand interface
- **Automatic Mode Switching**: System automatically switches to AUTO_MISSION mode and arms vehicle
- **No Automatic RTL**: System does not automatically trigger RTL after mission completion. User can set RTL waypoint as the last waypoint in the mission if needed.
- **MQTT Integration**: The system integrates with `mqtt_mission` node from `px4_mqtt` package:
  - `mqtt_mission` publishes waypoints to `/mission/waypoints` and trigger to `/px4_mission/trigger`
  - `px4_mission` publishes state to `/px4_mission/state` for `mqtt_mission` to forward to MQTT
- **Backward Compatibility**: `px4_mission` subscribes to both `/px4_mission/trigger` (primary) and `/mission/trigger` (legacy) for backward compatibility
- **QoS Configuration**: 
  - `/mission/waypoints` uses Reliable QoS (matching mavlink_mission subscription)
  - `/px4_mission/trigger` uses BestEffort QoS (matching px4_mission subscription)
  - `/px4_mission/state` uses default QoS (BestEffort)
- **GPS Requirement**: GPS must be available and home position must be set before sending missions
- **Status Display**: Status information is displayed at 2Hz when mission is active (ARMING, MISSION_ACTIVE, MISSION_COMPLETE). IDLE state is suppressed to reduce output clutter.
- **MAVLink Connection**: Default is `udp:127.0.0.1:14540` (configurable via ROS2 parameter `mavlink_connection`)
- **Mission Progress Tracking**: Uses `/mission/count` and `/mission/current_waypoint` topics from `mavlink_mission` instead of `mission_result` topic (which may not be available in all PX4 configurations)

（中文注释）
重要说明。

#### Troubleshooting

- **No waypoints received**: 
  - Check that `mavlink_mission.py` is running: `ros2 node list | grep mavlink_mission`
  - Verify waypoints are published: `ros2 topic echo /mission/waypoints`
  
- **Mission not uploading**: 
  - Check MAVLink connection: Verify PX4 is running and listening on `udp:127.0.0.1:14540`
  - Check logs: `ros2 topic echo /rosout | grep mavlink_mission`
  
- **Mission not starting**: 
  - Ensure GPS is available: Check `/fmu/out/vehicle_global_position`
  - Verify mission was uploaded: Check `/mission/count` topic (should show total waypoints)
  - Check MAVLink connection: Verify `mavlink_mission.py` is connected and uploaded successfully
  - Ensure vehicle is not in failsafe mode
  - Verify trigger was sent: Check `/px4_mission/trigger` or `/mission/trigger` topic
  
- **Status not updating**: 
  - Verify `px4_mission` node is running: `ros2 node list | grep px4_mission`
  - Check PX4 topics are available: `ros2 topic list | grep fmu`

（中文注释）
故障排除。

## File Structure

（中文注释）
文件结构总览：列出本包的头文件、源码、脚本、launch 与示例文件，方便快速定位实现与使用示例。

```
px4_control/
├── include/
│   ├── px4_control/
│   │   ├── px4_estimator.hpp         # Estimator header file
│   │   └── px4_control.hpp           # Control node header file
|   |   └── px4_servo_controller.hpp  #  
│   └── px4_mission/
│   |    └── px4_mission.hpp            # PX4 mission header file
│   └── px4_joystick/
│       ├── joystick_bridge.hpp       # Joystick bridge header file
│       └── px4_joystick.hpp          # PX4 joystick converter header file
├── src/
│   ├── px4_estimator.cpp             # Estimator implementation
│   ├── px4_estimator_node.cpp       # Estimator main entry point
│   ├── px4_control.cpp               # Control node implementation
│   └── px4_control_node.cpp         # Control node main entry point
├── utils/
│   ├── joystick_bridge.cpp           # Joystick bridge implementation
│   └── px4_joystick.cpp              # PX4 joystick converter implementation
│   ├── px4_mission.cpp               # PX4 mission implementation
│   └── px4_mission_node.cpp          # PX4 mission main entry point
├── launch/
│   ├── px4_estimator.launch.py      # Estimator launch file
│   ├── px4_control.launch.py        # Control node launch file
│   ├── px4_joystick.launch.py      # Complete joystick control launch file
│   └── px4_mission.launch.py        # PX4 mission planning system launch file
├── scripts/
│   ├── px4_control_cli.py           # Command-line interface script
│   ├── px4_mission_cli.py           # Mission command-line tool
│   └── mavlink_mission.py            # MAVLink mission uploader node
├── examples/
│   └── example_mission.json         # Example waypoint mission file
├── sh/
│   └── px4_control.sh                # One-click startup script
├── CMakeLists.txt
└── package.xml
```
