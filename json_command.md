{
  "uav_id": "uav1",                 无人机标识
  "uav_name": "uav1",
  "armed": false,                   是否解锁（true/false）
  "arming_state": 1,                解锁状态（0=初始化, 1=未解锁, 2=已解锁）
  "nav_state": 4,                   导航状态
  "battery_v": 12.6,                电池电压（V）
  "battery_b": 85.5,                电池电量百分比（%）
  "battery_current": 2.3,           电池电流（A）
  "position": [1.2, 3.4, 5.6],      本地位置 [x, y, z]（米）ENU坐标系待确定？
  "velocity": [0.1, 0.2, 0.0],      速度 [vx, vy, vz]（米/秒）
  "latitude": 39.1234567,           GPS位置（度，米）
  "longitude": 116.1234567,         GPS位置（度，米）
  "altitude": 100.5,                GPS位置（度，米）
  "dist_bottom": 10.5,              到地面距离（米）
  "attitude_rpy": [1.2, 3.4, 45.6], 姿态欧拉角（度）[roll, pitch, yaw]
  "attitude": [0.021, 0.059, 0.796],姿态欧拉角（弧度）[roll, pitch, yaw]
  "quaternion":四元数 [w, x, y, z]
  "gps_status": 3,                  GPS定位类型（0=无GPS, 1=无定位, 2=2D定位, 3=3D定位）
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
  "imu": {                          IMU数据（陀螺仪和加速度计）
    "gyro": [0.001, 0.002, 0.003],
    "accel": [0.1, 0.2, 9.8]
  },
  "timestamp": 1234567890123456789
}

{"command":"arm"}
{"command":"disarm"}
{"command":"set_mode_offboard"}
{"command":"set_mode_land"}
{"command":"set_mode_loiter"}
{"command":"set_mode_hold"}
{"command":"set_mode_auto"}
{"command":"set_mode_auto_rtl"}
{"command":"land"}
{"command":"takeoff","altitude":5.0}
{"command":"arm_and_takeoff","altitude":5.0}
{
  "command":"position",
  "x":10.0,
  "y":5.0,
  "z":3.0,
  "yaw":0.0
}
{
  "command":"velocity",
  "vx":1.0,
  "vy":0.0,
  "vz":0.0,
  "yaw":0.0
}
{
  "command":"velocity_body",
  "vx":1.0,
  "vy":0.0,
  "vz":0.0,
  "yaw_speed":0.0
}
