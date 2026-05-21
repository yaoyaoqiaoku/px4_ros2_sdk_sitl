#include "px4_control/px4_control.hpp"
#include <cmath>

PX4ControlNode::PX4ControlNode() : Node("px4_control")
{
    // ---------- QoS 配置 ----------
    // 命令类消息：必须可靠送达
    rclcpp::QoS command_qos(10);
    command_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    command_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // 动态状态/遥测：追求实时性，使用 BestEffort
    rclcpp::QoS status_qos(10);
    status_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    status_qos.history(rclcpp::HistoryPolicy::KeepLast);

    rclcpp::QoS default_qos(10);
    default_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    default_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // 发布器 ★ 离板控制模式与轨迹必须使用 Reliable
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", command_qos);
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", command_qos);   // 改用 Reliable
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", command_qos);     // 改用 Reliable

    // 订阅器
    vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/fmu/out/vehicle_command_ack", status_qos,
        std::bind(&PX4ControlNode::vehicle_command_ack_callback, this, std::placeholders::_1));
    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", default_qos,
        std::bind(&PX4ControlNode::vehicle_local_position_callback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", status_qos,
        std::bind(&PX4ControlNode::vehicle_status_callback, this, std::placeholders::_1));
    trajectory_setpoint_command_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_control/trajectory_setpoint_command", default_qos,
        std::bind(&PX4ControlNode::trajectory_setpoint_command_callback, this, std::placeholders::_1));
    vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/px4_control/vehicle_command", default_qos,
        std::bind(&PX4ControlNode::vehicle_command_callback, this, std::placeholders::_1));

    offboard_timer_ = this->create_wall_timer(50ms,
        std::bind(&PX4ControlNode::offboard_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "PX4 控制节点已启动 (修复版 v4)");
}

// =====================================================================
// 回调：VehicleCommandAck
// =====================================================================
void PX4ControlNode::vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
        if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            RCLCPP_INFO(this->get_logger(), "解锁/上锁命令执行成功");
            if (takeoff_pending_) {
                takeoff_pending_ = false;
                if (arm_timeout_timer_) {
                    arm_timeout_timer_->cancel();
                    arm_timeout_timer_.reset();
                }
                set_mode_auto_takeoff();
                RCLCPP_INFO(this->get_logger(), "解锁成功，自动切换到 AUTO.TAKEOFF");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "解锁/上锁命令失败，结果: %u", msg->result);
            if (takeoff_pending_) {
                takeoff_pending_ = false;
                if (arm_timeout_timer_) {
                    arm_timeout_timer_->cancel();
                    arm_timeout_timer_.reset();
                }
                RCLCPP_ERROR(this->get_logger(), "解锁失败，取消起飞请求");
            }
        }
    } else if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
        if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            RCLCPP_WARN(this->get_logger(), "模式切换命令失败，结果: %u", msg->result);
        }
    } else if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND) {
        if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            RCLCPP_WARN(this->get_logger(), "着陆命令失败，结果: %u", msg->result);
        }
    }
}

// =====================================================================
// 回调：VehicleLocalPosition
// =====================================================================
void PX4ControlNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_local_position_ = *msg;
    vehicle_local_position_received_ = true;

    if (offboard_mode_active_ && !setpoint_initialized_ &&
        msg->xy_valid && msg->z_valid &&
        !std::isnan(msg->heading))
    {
        current_pos_setpoint_[0] = msg->x;
        current_pos_setpoint_[1] = msg->y;
        current_pos_setpoint_[2] = msg->z;
        current_yaw_setpoint_ = msg->heading;
        setpoint_initialized_ = true;
        use_position_control_ = true;
        RCLCPP_INFO(this->get_logger(), "用有效位置初始化离板设定值");
    }
}

// =====================================================================
// 回调：VehicleStatus ★ 进入 OFFBOARD 时应用缓存的外部设定值（含超时检查）
// =====================================================================
void PX4ControlNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_status_ = *msg;
    vehicle_status_received_ = true;

    bool is_offboard = (vehicle_status_.nav_state == 14);

    if (is_offboard && !offboard_mode_detected_) {
        // 刚进入 OFFBOARD 模式
        offboard_mode_detected_ = true;
        offboard_mode_active_ = true;

        if (offboard_switch_timer_) {
            offboard_switch_timer_->cancel();
            offboard_switch_timer_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "飞行器已进入 OFFBOARD 模式");

        // 如果存在待处理的外部设定值，检查超时后应用
        if (pending_external_setpoint_) {
            double age = (this->get_clock()->now() - pending_setpoint_time_).seconds();
            if (age > PENDING_SETPOINT_TIMEOUT) {
                RCLCPP_WARN(this->get_logger(), "缓存的设定值已超时(%.1fs)，丢弃", age);
                pending_external_setpoint_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "应用缓存的待处理外部设定值(年龄: %.1fs)", age);
                if (ext_use_position_) {
                    for (int i = 0; i < 3; ++i)
                        current_pos_setpoint_[i] = ext_setpoint_pos_[i];
                    if (!std::isnan(ext_setpoint_yaw_))
                        current_yaw_setpoint_ = ext_setpoint_yaw_;
                    current_vel_setpoint_[0] = current_vel_setpoint_[1] = current_vel_setpoint_[2] = 0.0f;
                    use_position_control_ = true;
                } else {
                    for (int i = 0; i < 3; ++i)
                        current_vel_setpoint_[i] = ext_setpoint_vel_[i];
                    if (!std::isnan(ext_setpoint_yawspeed_))
                        current_yawspeed_setpoint_ = ext_setpoint_yawspeed_;
                    use_position_control_ = false;
                }
                setpoint_initialized_ = true;
                pending_external_setpoint_ = false;
                // 立即发布一次以应用
                publish_offboard_setpoint_impl();
            }
        }
        // 若无外部设定值，则用默认悬停逻辑（已在 offboard_timer_callback 中处理）

    } else if (!is_offboard && offboard_mode_detected_) {
        offboard_mode_detected_ = false;
        offboard_mode_active_ = false;
        offboard_handshake_active_ = false;
        if (offboard_switch_timer_) {
            offboard_switch_timer_->cancel();
            offboard_switch_timer_.reset();
        }
    }
}

// =====================================================================
// 回调：外部模式切换命令
// =====================================================================
void PX4ControlNode::vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg)
{
    if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
        msg->param1 == 1.0f)
    {
        uint8_t main_mode = static_cast<uint8_t>(msg->param2);
        uint8_t sub_mode = static_cast<uint8_t>(msg->param3);
        RCLCPP_INFO(this->get_logger(), "收到 DO_SET_MODE 请求: main=%u sub=%u", main_mode, sub_mode);

        switch (main_mode) {
            case 1: set_mode_manual(); return;
            case 2: set_mode_altctl(); return;
            case 3:
                if (sub_mode == 1) set_mode_posctl();
                else if (sub_mode == 2) set_mode_posctl_orbit();
                else set_mode_posctl();
                return;
            case 4:
                switch (sub_mode) {
                    case 1: RCLCPP_WARN(this->get_logger(), "AUTO.READY 忽略"); return;
                    case 2: set_mode_auto_takeoff(); return;
                    case 3: set_mode_auto_loiter(); return;
                    case 4: set_mode_auto_mission(); return;
                    case 5: set_mode_auto_rtl(); return;
                    case 6: set_mode_land(); return;
                    case 9: set_mode_auto_precland(); return;
                    case 10: set_mode_auto_vtol_takeoff(); return;
                    default:
                        if (sub_mode >= 11 && sub_mode <= 18) {
                            set_mode_external(sub_mode - 10);
                            return;
                        }
                        RCLCPP_WARN(this->get_logger(), "未知 AUTO 子模式: %u", sub_mode);
                        return;
                }
                break;
            case 5: set_mode_acro(); return;
            case 6: set_mode_offboard(); return;
            case 7: set_mode_stabilized(); return;
            default:
                RCLCPP_WARN(this->get_logger(), "未知 main_mode: %u，直接转发", main_mode);
                break;
        }
    }

    // 直接转发
    px4_msgs::msg::VehicleCommand forward_msg = *msg;
    forward_msg.timestamp = get_timestamp();
    vehicle_command_pub_->publish(forward_msg);
}

// =====================================================================
// 回调：外部轨迹设定值 ★ 修复 Bug1（安全握手 + 缓存设定值 + 超时记录）
// =====================================================================
void PX4ControlNode::trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
{
    bool need_offboard_switch = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 解析设定值有效性
        bool pos_valid[3];
        for (int i = 0; i < 3; ++i) pos_valid[i] = !std::isnan(msg->position[i]);
        bool has_position = pos_valid[0] && pos_valid[1] && pos_valid[2];

        bool has_velocity = false;
        for (int i = 0; i < 3; ++i)
            if (!std::isnan(msg->velocity[i])) has_velocity = true;

        if (has_position && has_velocity) {
            RCLCPP_WARN(this->get_logger(), "同时包含位置和速度，优先位置控制");
            has_velocity = false;
        }
        if (!has_position && !has_velocity) {
            RCLCPP_WARN(this->get_logger(), "无效轨迹设定值");
            return;
        }

        // 若已在 OFFBOARD 模式，直接应用并发布
        if (offboard_mode_detected_) {
            if (has_position) {
                for (int i = 0; i < 3; ++i) current_pos_setpoint_[i] = msg->position[i];
                current_vel_setpoint_[0] = current_vel_setpoint_[1] = current_vel_setpoint_[2] = 0.0f;
                if (!std::isnan(msg->yaw)) current_yaw_setpoint_ = msg->yaw;
                use_position_control_ = true;
            } else {
                for (int i = 0; i < 3; ++i)
                    current_vel_setpoint_[i] = std::isnan(msg->velocity[i]) ? 0.0f : msg->velocity[i];
                if (!std::isnan(msg->yawspeed)) current_yawspeed_setpoint_ = msg->yawspeed;
                else if (!std::isnan(msg->yaw)) {
                    current_yawspeed_setpoint_ = msg->yaw;
                    RCLCPP_WARN(this->get_logger(), "速度设定值 yaw 被解释为偏航角速率");
                }
                use_position_control_ = false;
            }
            setpoint_initialized_ = true;
            publish_offboard_setpoint_impl();
            return;
        }

        // 未进入 OFFBOARD：缓存设定值，记录时间，待模式切换完成后应用
        RCLCPP_INFO(this->get_logger(), "未进入 OFFBOARD，缓存设定值并请求模式切换");
        if (has_position) {
            for (int i = 0; i < 3; ++i) ext_setpoint_pos_[i] = msg->position[i];
            ext_setpoint_yaw_ = msg->yaw;
            ext_use_position_ = true;
        } else {
            for (int i = 0; i < 3; ++i) ext_setpoint_vel_[i] = msg->velocity[i];
            ext_setpoint_yawspeed_ = msg->yawspeed;
            ext_use_position_ = false;
        }
        pending_external_setpoint_ = true;
        pending_setpoint_time_ = this->get_clock()->now();   // 记录时间戳
        need_offboard_switch = true;
    } // 锁释放

    if (need_offboard_switch) {
        set_mode_offboard();  // 内部会自己加锁，启动安全握手
    }
}

// =====================================================================
// offboard_timer_callback ★ 握手阶段不再发送危险零位值
// =====================================================================
void PX4ControlNode::offboard_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (offboard_handshake_active_) {
        if (setpoint_initialized_) {
            publish_offboard_setpoint_impl();
        } else {
            // 无有效设定值时，绝不可发送零位，立即取消握手，防止炸机
            RCLCPP_FATAL(this->get_logger(), "握手阶段无有效位置，紧急取消离板模式切换");
            offboard_handshake_active_ = false;
            offboard_mode_active_ = false;
            if (offboard_switch_timer_) {
                offboard_switch_timer_->cancel();
                offboard_switch_timer_.reset();
            }
            return;
        }
        offboard_handshake_count_++;
        if (offboard_handshake_count_ >= HANDSHAKE_REQUIRED) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                                    1.0f, 6.0f, 0.0f);
            offboard_handshake_active_ = false;
            offboard_handshake_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "离板握手完成，已请求切换至 OFFBOARD");
        }
        return;
    }

    if (offboard_mode_active_ || offboard_mode_detected_) {
        if (offboard_mode_detected_ && !offboard_mode_active_) offboard_mode_active_ = true;

        if (setpoint_initialized_) {
            publish_offboard_setpoint_impl();
        }
        else if (vehicle_local_position_received_ &&
                 vehicle_local_position_.xy_valid && vehicle_local_position_.z_valid &&
                 !std::isnan(vehicle_local_position_.heading))
        {
            current_pos_setpoint_[0] = vehicle_local_position_.x;
            current_pos_setpoint_[1] = vehicle_local_position_.y;
            current_pos_setpoint_[2] = vehicle_local_position_.z;
            current_yaw_setpoint_  = vehicle_local_position_.heading;
            current_vel_setpoint_[0] = current_vel_setpoint_[1] = current_vel_setpoint_[2] = 0.0f;
            use_position_control_ = true;
            setpoint_initialized_ = true;
            publish_offboard_setpoint_impl();
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "无有效位置数据，无法自动悬停！立即尝试降落");
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f, 4.0f, 6.0f);
            offboard_mode_active_ = false;
            if (offboard_switch_timer_) {
                offboard_switch_timer_->cancel();
                offboard_switch_timer_.reset();
            }
        }
    }
}

// =====================================================================
// 内部：发布离板设定值（需持有 state_mutex_）
// =====================================================================
void PX4ControlNode::publish_offboard_setpoint_impl()
{
    if (use_position_control_) {
        publish_offboard_control_mode(true, false, false, false, false);
        publish_trajectory_setpoint(
            current_pos_setpoint_[0], current_pos_setpoint_[1], current_pos_setpoint_[2],
            0,0,0,
            current_yaw_setpoint_, std::nanf(""));
    } else {
        publish_offboard_control_mode(false, true, false, false, false);
        publish_trajectory_setpoint(
            std::nanf(""), std::nanf(""), std::nanf(""),
            current_vel_setpoint_[0], current_vel_setpoint_[1], current_vel_setpoint_[2],
            std::nanf(""), current_yawspeed_setpoint_);
    }
}

// =====================================================================
// arm_and_takeoff
// =====================================================================
void PX4ControlNode::arm_and_takeoff()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_INFO(this->get_logger(), "解锁并准备自动起飞...");

    if (vehicle_status_received_ &&
        vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        set_mode_auto_takeoff();
        return;
    }

    takeoff_pending_ = true;
    arm();

    arm_timeout_timer_ = this->create_wall_timer(5000ms, [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (takeoff_pending_) {
            RCLCPP_ERROR(this->get_logger(), "解锁超时，取消起飞请求");
            takeoff_pending_ = false;
        }
        if (arm_timeout_timer_) {
            arm_timeout_timer_->cancel();
            arm_timeout_timer_.reset();
        }
    });
}

// =====================================================================
// set_mode_offboard ★ 加入 Home 点有效性检查
// =====================================================================
void PX4ControlNode::set_mode_offboard()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_INFO(this->get_logger(), "启动离板握手流程...");

    if (!vehicle_local_position_received_ ||
        !vehicle_local_position_.xy_valid || !vehicle_local_position_.z_valid ||
        std::isnan(vehicle_local_position_.heading))
    {
        RCLCPP_ERROR(this->get_logger(), "无有效位置数据，无法进入 OFFBOARD 模式！");
        return;
    }

    // ★ 新增：Home 点有效性检查（使用 xy_global_valid 和 z_global_valid）
    if (!vehicle_local_position_.xy_global || !vehicle_local_position_.z_global) {
        RCLCPP_ERROR(this->get_logger(), "Home点未校准，无法进入 OFFBOARD 模式");
        return;
    }

    current_pos_setpoint_[0] = vehicle_local_position_.x;
    current_pos_setpoint_[1] = vehicle_local_position_.y;
    current_pos_setpoint_[2] = vehicle_local_position_.z;
    current_yaw_setpoint_ = vehicle_local_position_.heading;
    setpoint_initialized_ = true;
    use_position_control_ = true;

    offboard_handshake_active_ = true;
    offboard_handshake_count_ = 0;
    offboard_switch_attempts_ = 0;

    offboard_switch_timer_ = this->create_wall_timer(
        100ms, std::bind(&PX4ControlNode::check_offboard_switch_status, this));
}

// =====================================================================
// 检查 OFFBOARD 切换状态
// =====================================================================
void PX4ControlNode::check_offboard_switch_status()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (vehicle_status_.nav_state == 14) {
        RCLCPP_INFO(this->get_logger(), "成功进入 OFFBOARD 模式");
        if (offboard_switch_timer_) {
            offboard_switch_timer_->cancel();
            offboard_switch_timer_.reset();
        }
        offboard_handshake_active_ = false;
        return;
    }

    if (offboard_handshake_active_) {
        return;
    }

    if (++offboard_switch_attempts_ > MAX_OFFBOARD_SWITCH_ATTEMPTS) {
        RCLCPP_ERROR(this->get_logger(), "进入 OFFBOARD 模式失败，已重试 %d 次，放弃",
                     MAX_OFFBOARD_SWITCH_ATTEMPTS);
        offboard_mode_active_ = false;
        offboard_handshake_active_ = false;
        if (offboard_switch_timer_) {
            offboard_switch_timer_->cancel();
            offboard_switch_timer_.reset();
        }
        return;
    }

    RCLCPP_WARN(this->get_logger(), "OFFBOARD 未成功，重试发送 DO_SET_MODE（第 %d 次）",
                offboard_switch_attempts_);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 6.0f, 0.0f);
}

// =====================================================================
// 其他公共函数
// =====================================================================
void PX4ControlNode::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}
void PX4ControlNode::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PX4ControlNode::set_mode_land() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 4.0f, 6.0f);
}
void PX4ControlNode::set_mode_hold() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 4.0f, 3.0f);
}
void PX4ControlNode::set_mode_manual() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 1.0f, 0.0f);
}
void PX4ControlNode::set_mode_altctl() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 2.0f, 0.0f);
}
void PX4ControlNode::set_mode_posctl() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f, 0.0f);
}
void PX4ControlNode::set_mode_posctl_orbit() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f, 1.0f);
}
void PX4ControlNode::set_mode_acro() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 5.0f, 0.0f);
}
void PX4ControlNode::set_mode_stabilized() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 7.0f, 0.0f);
}
void PX4ControlNode::set_mode_auto_mission() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 4.0f);
}
void PX4ControlNode::set_mode_auto_loiter() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 3.0f);
}
void PX4ControlNode::set_mode_auto_rtl() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 5.0f);
}
void PX4ControlNode::set_mode_auto_precland() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 9.0f);
}
void PX4ControlNode::set_mode_auto_vtol_takeoff() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 10.0f);
}
void PX4ControlNode::set_mode_external(uint8_t ext_index) {
    if (ext_index < 1 || ext_index > 8) {
        RCLCPP_WARN(this->get_logger(), "外部模式索引无效: %u", ext_index);
        return;
    }
    float sub_mode = static_cast<float>(10 + ext_index);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 4.0f, sub_mode);
}
void PX4ControlNode::set_actuator_outputs(const std::array<float,6>& values, int index) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
                            values[0], values[1], values[2], values[3],
                            static_cast<double>(values[4]), static_cast<double>(values[5]),
                            static_cast<float>(index));
}
void PX4ControlNode::set_servo_pwm(uint8_t servo_number, float pwm_usec) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPEAT_SERVO,
                            static_cast<float>(servo_number), pwm_usec, 1.0f, 0.1f);
}

// ---------------------------------------------------------------------------
// 发布离板控制模式/轨迹
// ---------------------------------------------------------------------------
void PX4ControlNode::publish_offboard_control_mode(bool position, bool velocity,
                                                   bool acceleration, bool attitude,
                                                   bool body_rate)
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = get_timestamp();
    msg.position = position;
    msg.velocity = velocity;
    msg.acceleration = acceleration;
    msg.attitude = attitude;
    msg.body_rate = body_rate;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
}

void PX4ControlNode::publish_trajectory_setpoint(float x, float y, float z,
                                                 float vx, float vy, float vz,
                                                 float yaw, float yawspeed)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = get_timestamp();
    msg.position[0] = x; msg.position[1] = y; msg.position[2] = z;
    msg.velocity[0] = vx; msg.velocity[1] = vy; msg.velocity[2] = vz;
    msg.acceleration[0] = std::nanf("");
    msg.acceleration[1] = std::nanf("");
    msg.acceleration[2] = std::nanf("");
    msg.yaw = yaw;
    msg.yawspeed = yawspeed;
    trajectory_setpoint_pub_->publish(msg);
}

// ★ 死锁修复版
void PX4ControlNode::publish_position_setpoint(float x, float y, float z, float yaw)
{
    bool need_enter_offboard = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        need_enter_offboard = (!offboard_mode_active_ && vehicle_status_received_ &&
                               vehicle_status_.nav_state != 14);
        current_pos_setpoint_[0] = x;
        current_pos_setpoint_[1] = y;
        current_pos_setpoint_[2] = z;
        current_yaw_setpoint_ = yaw;
        current_vel_setpoint_[0] = current_vel_setpoint_[1] = current_vel_setpoint_[2] = 0.0f;
        use_position_control_ = true;
        setpoint_initialized_ = true;
        offboard_mode_active_ = true;
    }
    if (need_enter_offboard) {
        set_mode_offboard();
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        publish_offboard_setpoint_impl();
    }
}

void PX4ControlNode::publish_velocity_setpoint(float vx, float vy, float vz, float yawspeed)
{
    bool need_enter_offboard = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        need_enter_offboard = (!offboard_mode_active_ && vehicle_status_received_ &&
                               vehicle_status_.nav_state != 14);
        current_vel_setpoint_[0] = vx;
        current_vel_setpoint_[1] = vy;
        current_vel_setpoint_[2] = vz;
        current_yawspeed_setpoint_ = yawspeed;
        use_position_control_ = false;
        setpoint_initialized_ = true;
        offboard_mode_active_ = true;
    }
    if (need_enter_offboard) {
        set_mode_offboard();
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        publish_offboard_setpoint_impl();
    }
}

void PX4ControlNode::publish_body_velocity_setpoint(float vx_b, float vy_b, float vz_b,
                                                    float yaw, float yawspeed)
{
    float yaw_used = yaw;
    if (std::isnan(yaw_used)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (vehicle_local_position_received_ &&
            !std::isnan(vehicle_local_position_.heading))
            yaw_used = vehicle_local_position_.heading;
        else
            yaw_used = current_yaw_setpoint_;
    }
    float cy = std::cos(yaw_used);
    float sy = std::sin(yaw_used);
    float vx_n = cy * vx_b - sy * vy_b;
    float vy_n = sy * vx_b + cy * vy_b;
    float vz_n = vz_b;
    publish_velocity_setpoint(vx_n, vy_n, vz_n, yawspeed);
}

// ---------------------------------------------------------------------------
// 辅助
// ---------------------------------------------------------------------------
uint64_t PX4ControlNode::get_timestamp() {
    return this->get_clock()->now().nanoseconds() / 1000ULL;
}

void PX4ControlNode::publish_vehicle_command(uint32_t command,
                                             float param1, float param2,
                                             float param3, float param4,
                                             double param5, double param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = get_timestamp();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.confirmation = 0;
    vehicle_command_pub_->publish(msg);
}

void PX4ControlNode::set_mode_auto_takeoff() {
    // ★ 起飞前检查 Home 点有效性
    if (!vehicle_local_position_received_ ||
        !vehicle_local_position_.xy_global || !vehicle_local_position_.z_global) {
        RCLCPP_ERROR(this->get_logger(), "Home点未校准，无法执行自动起飞");
        return;
    }
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 4.0f, 2.0f);
}