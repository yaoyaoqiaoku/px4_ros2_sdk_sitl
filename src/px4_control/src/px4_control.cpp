/****************************************************************************
 *
 * PX4 控制节点实现
 * 提供解锁/上锁、解锁+起飞和离板控制（位置/速度）功能
 *
 ****************************************************************************/
// --- 中文说明 -----------------------------------------------------------
// 本文件实现 PX4ControlNode 的具体逻辑。主要包含：
// - 订阅/发布接口的创建（与 PX4 交互的主题，如 /fmu/in/vehicle_command）
// - OFFBOARD 模式维护逻辑（定时器周期性发布 offboard_control_mode 与 trajectory_setpoint）
// - 对外接收的 VehicleCommand 进行预处理或转发
//
// 关键函数说明：
// - PX4ControlNode::PX4ControlNode: 初始化节点、QoS、发布器/订阅器与定时器
// - vehicle_command_ack_callback: 处理来自 PX4 的命令应答（只对错误进行警告）
// - vehicle_command_callback: 接收来自外部的命令（例如 MQTT）并根据类型进行处理/转发
// - set_mode_offboard: 进入 OFFBOARD 模式前的初始化和启动握手（需要多次下发设定值）
// - offboard_timer_callback: 定时发布维持 OFFBOARD 模式所需的消息
// ------------------------------------------------------------------------

#include "px4_control/px4_control.hpp"
#include <thread>
#include <chrono>
#include <cmath>

// 构造函数：初始化节点、QoS、发布器/订阅器、以及维持 OFFBOARD 的定时器
PX4ControlNode::PX4ControlNode() : Node("px4_control")
{
	// 根据 PX4 DDS 要求配置 QoS 配置文件
	rclcpp::QoS status_qos(10);
	status_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	status_qos.history(rclcpp::HistoryPolicy::KeepLast);

	rclcpp::QoS default_qos(10);
	default_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	default_qos.history(rclcpp::HistoryPolicy::KeepLast);

	// 创建发布器
	vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
		"/fmu/in/vehicle_command", 10);
	
	offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
		"/fmu/in/offboard_control_mode", 10);
	
	trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"/fmu/in/trajectory_setpoint", 10);

	// 创建订阅器
	vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
		"/fmu/out/vehicle_command_ack", status_qos,
		std::bind(&PX4ControlNode::vehicle_command_ack_callback, this, std::placeholders::_1));

	vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", default_qos,
		std::bind(&PX4ControlNode::vehicle_local_position_callback, this, std::placeholders::_1));

	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status", status_qos,
		std::bind(&PX4ControlNode::vehicle_status_callback, this, std::placeholders::_1));

	// 订阅轨迹设定值命令（来自命令行或其他节点）
	trajectory_setpoint_command_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
		"/px4_control/trajectory_setpoint_command", default_qos,
		std::bind(&PX4ControlNode::trajectory_setpoint_command_callback, this, std::placeholders::_1));

	// 订阅飞行器命令请求（来自 MQTT 或其他节点）
	vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
		"/px4_control/vehicle_command", default_qos,
		std::bind(&PX4ControlNode::vehicle_command_callback, this, std::placeholders::_1));

	// 创建离板模式维护定时器（必须至少 2Hz 发布）
	// 使用 50ms = 20Hz 确保超过 2Hz 要求
	offboard_timer_ = this->create_wall_timer(50ms, std::bind(&PX4ControlNode::offboard_timer_callback, this));

	// 初始化设定值变量
	current_pos_setpoint_[0] = 0.0f;
	current_pos_setpoint_[1] = 0.0f;
	current_pos_setpoint_[2] = 0.0f;
	current_vel_setpoint_[0] = 0.0f;
	current_vel_setpoint_[1] = 0.0f;
	current_vel_setpoint_[2] = 0.0f;
	current_yaw_setpoint_ = 0.0f;
	use_position_control_ = true;
	setpoint_initialized_ = false;

	RCLCPP_INFO(this->get_logger(), "PX4 控制节点已启动");
}

// 处理来自 PX4 的 VehicleCommandAck
// 仅在失败或需要关注时打印日志（避免信息泛滥）
void PX4ControlNode::vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg)
{
	// 仅对错误情况发出警告
	if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
		if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
			RCLCPP_WARN(this->get_logger(), "解锁/上锁命令失败，结果: %u", msg->result);
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

// 接收外部 VehicleCommand（例如来自 MQTT 桥接或控制界面）
// 对某些需要本地处理的命令（如请求切换 OFFBOARD）进行特殊处理，否则直接转发给 PX4
void PX4ControlNode::vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg)
{
	
	// 检查是否是切换模式的命令（DO_SET_MODE）并路由到具体的模式处理函数
	if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
		msg->param1 == 1.0f) {
		// param2 = main_mode, param3 = sub_mode
		uint8_t main_mode = static_cast<uint8_t>(msg->param2);
		uint8_t sub_mode = static_cast<uint8_t>(msg->param3);

		RCLCPP_INFO(this->get_logger(), "收到 DO_SET_MODE 请求: main=%u sub=%u", main_mode, sub_mode);

		switch (main_mode) {
			case 1: // MANUAL
				set_mode_manual();
				return;
			case 2: // ALTCTL
				set_mode_altctl();
				return;
			case 3: // POSCTL
				// POSCTL 可能有子模式（ORBIT / SLOW）
				if (sub_mode == 1) {
					// POSCTL default
					set_mode_posctl();
				} else if (sub_mode == 2) {
					set_mode_posctl_orbit();
				} else if (sub_mode == 3) {
					// SLOW
					set_mode_posctl();
				} else {
					set_mode_posctl();
				}
				return;
			case 4: // AUTO main mode, 子模式很多（TAKEOFF, LOITER, MISSION, RTL, LAND, EXTERNALx）
				switch (sub_mode) {
					case 1: set_mode_auto_mission(); return; // READY -> treat as mission
					case 2: set_mode_auto_takeoff(); return;
					case 3: set_mode_auto_loiter(); return;
					case 4: set_mode_auto_mission(); return;
					case 5: set_mode_auto_rtl(); return;
					case 6: set_mode_land(); return;
					case 9: set_mode_auto_precland(); return;
					case 10: set_mode_auto_vtol_takeoff(); return;
					default:
						if (sub_mode >= 11 && sub_mode <= 18) {
							// EXTERNAL1..EXTERNAL8 (11..18)
							uint8_t ext_index = sub_mode - 10; // map 11->1
							set_mode_external(ext_index);
							return;
						}
						// fallback
						set_mode_auto_mission();
						return;
				}
				break;
			case 5: // ACRO
				set_mode_acro();
				return;
			case 6: // OFFBOARD
				// 使用原先的离板初始化流程
				set_mode_offboard();
				return;
			case 7: // STABILIZED
				set_mode_stabilized();
				return;
			default:
				RCLCPP_WARN(this->get_logger(), "未知的 main_mode: %u，直接转发命令", main_mode);
				break;
		}
		// 如果没有 return，则继续把原始命令转发
	}
	
	// 其他命令直接转发给 PX4
	px4_msgs::msg::VehicleCommand forward_msg = *msg;
	forward_msg.timestamp = get_timestamp();
	
	RCLCPP_DEBUG(this->get_logger(), "转发飞行器命令 %u 到 PX4", msg->command);
	vehicle_command_pub_->publish(forward_msg);
}

// 获取时间戳（微秒），用于填充 px4_msgs 的 timestamp 字段
uint64_t PX4ControlNode::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

// 构造并发布 VehicleCommand 到 /fmu/in/vehicle_command
// 参数语义遵循 MAVLink/px4_msgs 的定义（param1..param7 可用于不同命令）
void PX4ControlNode::publish_vehicle_command(uint32_t command, float param1, float param2,
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

// 发起解锁（ARM）动作，内部通过 publish_vehicle_command 填充对应命令
void PX4ControlNode::arm()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		1.0f
	);
}

// 发起上锁（DISARM）动作
void PX4ControlNode::disarm()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		0.0f
	);
}

// 切换到 AUTO.TAKEOFF 模式（通过 DO_SET_MODE 的参数组合）
void PX4ControlNode::set_mode_auto_takeoff()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 2.0f
	);
}

// 切换到 AUTO.LAND 模式
void PX4ControlNode::set_mode_land()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 6.0f
	);
}

// 切换到 AUTO.LOITER / HOLD 模式，用于悬停保持
void PX4ControlNode::set_mode_hold()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 3.0f
	);
}

// --- 补充的模式切换实现 --------------------------------------------------
void PX4ControlNode::set_mode_manual()
{
	RCLCPP_INFO(this->get_logger(), "切换到 MANUAL 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 1.0f, 0.0f);
}

void PX4ControlNode::set_mode_altctl()
{
	RCLCPP_INFO(this->get_logger(), "切换到 ALTCTL 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 2.0f, 0.0f);
}

void PX4ControlNode::set_mode_posctl()
{
	RCLCPP_INFO(this->get_logger(), "切换到 POSCTL 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 3.0f, 0.0f);
}

void PX4ControlNode::set_mode_posctl_orbit()
{
	RCLCPP_INFO(this->get_logger(), "切换到 POSCTL.ORBIT 模式");
	// 在 POSCTL 主模式下，sub_mode=1/2/3 表示不同子模式；这里用 1/2/3 的约定
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 3.0f, 1.0f);
}

void PX4ControlNode::set_mode_acro()
{
	RCLCPP_INFO(this->get_logger(), "切换到 ACRO 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 5.0f, 0.0f);
}

void PX4ControlNode::set_mode_stabilized()
{
	RCLCPP_INFO(this->get_logger(), "切换到 STABILIZED 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 7.0f, 0.0f);
}

void PX4ControlNode::set_mode_auto_mission()
{
	RCLCPP_INFO(this->get_logger(), "切换到 AUTO.MISSION 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, 4.0f);
}

void PX4ControlNode::set_mode_auto_loiter()
{
	RCLCPP_INFO(this->get_logger(), "切换到 AUTO.LOITER 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, 3.0f);
}

void PX4ControlNode::set_mode_auto_rtl()
{
	RCLCPP_INFO(this->get_logger(), "切换到 AUTO.RTL 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, 5.0f);
}

void PX4ControlNode::set_mode_auto_precland()
{
	RCLCPP_INFO(this->get_logger(), "切换到 AUTO.PRECLAND 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, 9.0f);
}

void PX4ControlNode::set_mode_auto_vtol_takeoff()
{
	RCLCPP_INFO(this->get_logger(), "切换到 AUTO.VTOL_TAKEOFF 模式");
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, 10.0f);
}

void PX4ControlNode::set_mode_external(uint8_t ext_index)
{
	if (ext_index < 1 || ext_index > 8) {
		RCLCPP_WARN(this->get_logger(), "外部模式索引无效: %u", ext_index);
		return;
	}
	RCLCPP_INFO(this->get_logger(), "切换到 EXTERNAL%u 模式", ext_index);
	// EXTERNAL1..EXTERNAL8 在 px4_custom_mode.h 中为 11..18
	float sub_mode = static_cast<float>(10 + ext_index);
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
							1.0f, 4.0f, sub_mode);
}

// 设置 actuator outputs（DO_SET_ACTUATOR）
void PX4ControlNode::set_actuator_outputs(const std::array<float,6>& values, int index)
{
	RCLCPP_INFO(this->get_logger(), "发布 actuator outputs 到 index=%d", index);
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
							values[0], values[1], values[2], values[3],
							static_cast<double>(values[4]), static_cast<double>(values[5]),
							static_cast<float>(index));
}

// 便捷：设置单个舵机 PWM（采用 DO_REPEAT_SERVO 或 DO_SET_ACTUATOR）。
// 这里用 DO_REPEAT_SERVO 作为简单接口：param1=servo_num, param2=pwm_usec, param3=cycle_count=1, param4=period
void PX4ControlNode::set_servo_pwm(uint8_t servo_number, float pwm_usec)
{
	RCLCPP_INFO(this->get_logger(), "设置舵机 %u 到 PWM %.0fus", servo_number, pwm_usec);
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPEAT_SERVO,
							static_cast<float>(servo_number), pwm_usec, 1.0f, 0.1f);
}

// -----------------------------------------------------------------------

// 便捷函数：先解锁（arm），等待短暂时间，然后发送 AUTO.TAKEOFF 请求
// 注意：实际项目中应等待 ACK 确认后再继续，此处为简化实现
void PX4ControlNode::arm_and_takeoff()
{
	RCLCPP_INFO(this->get_logger(), "正在解锁并切换到自动起飞模式...");
	// 步骤1：解锁飞行器
	arm();
	// 等待解锁完成（PX4 需要时间处理解锁命令）
	// 注意：实际应用中应等待 vehicle_command_ack 确认解锁成功
	// 此处简化为等待短时间
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	// 步骤2：切换到自动起飞模式
	set_mode_auto_takeoff();
}

// 接收本地位置更新（来自 /fmu/out/vehicle_local_position），并在首次进入 OFFBOARD 时
// 用当前位置初始化设定值，以便平滑过渡到离板控制
void PX4ControlNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
	vehicle_local_position_ = *msg;
	vehicle_local_position_received_ = true;
	
	// 如果处于离板模式但设定值未初始化，使用当前位置
	if (offboard_mode_active_ && !setpoint_initialized_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
		current_yaw_setpoint_ = vehicle_local_position_.heading;
		setpoint_initialized_ = true;
		use_position_control_ = true;
		RCLCPP_INFO(this->get_logger(), "使用当前位置初始化离板设定值: [%.2f, %.2f, %.2f]",
		            current_pos_setpoint_[0], current_pos_setpoint_[1], current_pos_setpoint_[2]);
	}
}

// 接收 VehicleStatus，并检测是否进入 OFFBOARD 模式（nav_state == 14）
// 由此触发内部状态切换（offboard_mode_detected_ / offboard_mode_active_）
void PX4ControlNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
	vehicle_status_ = *msg;
	vehicle_status_received_ = true;
	
	// 检查飞行器是否处于离板模式
	// PX4 vehicle_status.nav_state 取值：
	// 14 = 导航状态：离板模式
	bool is_offboard = (vehicle_status_.nav_state == 14);
	
	if (is_offboard && !offboard_mode_detected_) {
		// 刚进入离板模式
		offboard_mode_detected_ = true;
		offboard_mode_active_ = true;  // 启用发布
		
		// 如果位置可用且设定值未初始化，用当前位置初始化设定值
		if (vehicle_local_position_received_) {
			// 进入离板模式时始终更新设定值以保持当前位置
			if (!setpoint_initialized_ || !offboard_mode_active_) {
				current_pos_setpoint_[0] = vehicle_local_position_.x;
				current_pos_setpoint_[1] = vehicle_local_position_.y;
				current_pos_setpoint_[2] = vehicle_local_position_.z;
				current_yaw_setpoint_ = vehicle_local_position_.heading;
				setpoint_initialized_ = true;
				use_position_control_ = true;
			}
		}
		
		if (!offboard_mode_active_) {
			offboard_mode_active_ = true;
		}
	} else if (!is_offboard && offboard_mode_detected_) {
		offboard_mode_detected_ = false;
		offboard_mode_active_ = false;
	}
}

// 接收外部下发的 trajectory_setpoint（位置/速度），更新内部设定值并立即发布一次
// 定时器会继续以 20Hz 发布以维持 OFFBOARD
void PX4ControlNode::trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
{
	// 此回调接收来自命令行或其他节点的设定值命令
	// 更新内部设定值状态，以便定时器持续发布
	
	// 根据消息内容确定控制类型
	bool has_position = false;
	bool has_velocity = false;
	
	// 检查位置是否有效（非 NaN）- 三个分量都必须有效
	bool pos_valid[3] = {false, false, false};
	for (int i = 0; i < 3; ++i) {
		if (!std::isnan(msg->position[i])) {
			pos_valid[i] = true;
			current_pos_setpoint_[i] = msg->position[i];
		}
	}
	has_position = pos_valid[0] && pos_valid[1] && pos_valid[2];
	
	// 检查速度是否有效（非 NaN）- 至少一个分量有效
	for (int i = 0; i < 3; ++i) {
		if (!std::isnan(msg->velocity[i])) {
			has_velocity = true;
			current_vel_setpoint_[i] = msg->velocity[i];
		} else {
			current_vel_setpoint_[i] = 0.0f;
		}
	}
	
	// 更新偏航角
	if (!std::isnan(msg->yaw)) {
		current_yaw_setpoint_ = msg->yaw;
	}
	
	// 确定控制模式
	if (has_position && !has_velocity) {
		// 仅位置控制
		use_position_control_ = true;
		current_vel_setpoint_[0] = 0.0f;
		current_vel_setpoint_[1] = 0.0f;
		current_vel_setpoint_[2] = 0.0f;
	} else if (has_velocity) {
		use_position_control_ = false;
		if (vehicle_local_position_received_ && !has_position) {
			current_pos_setpoint_[0] = vehicle_local_position_.x;
			current_pos_setpoint_[1] = vehicle_local_position_.y;
			current_pos_setpoint_[2] = vehicle_local_position_.z;
		}
	} else {
		return;
	}
	
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	setpoint_initialized_ = true;
	
	// 立即发布一次（定时器会继续以 20Hz 发布）
	if (use_position_control_) {
		publish_offboard_control_mode(true, false, false, false, false);
		publish_trajectory_setpoint(
			current_pos_setpoint_[0],
			current_pos_setpoint_[1],
			current_pos_setpoint_[2],
			0.0f, 0.0f, 0.0f,
			current_yaw_setpoint_,
			0.0f
		);
	} else {
		float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
		publish_offboard_control_mode(false, true, false, false, false);
		publish_trajectory_setpoint(
			std::nanf(""), std::nanf(""), z,
			current_vel_setpoint_[0],
			current_vel_setpoint_[1],
			current_vel_setpoint_[2],
			current_yaw_setpoint_,
			0.0f
		);
	}
}

// 定时器回调：如果处于离板模式（或检测到离板），则持续发布 offboard_control_mode 与 trajectory_setpoint
// 确保发布频率满足 PX4 要求（>= 2Hz），本实现使用 20Hz
void PX4ControlNode::offboard_timer_callback()
{
	// 如果离板模式激活（手动激活或从飞行器状态检测到），
	// 通过发布控制模式和设定值维持该模式
	// PX4 要求 offboard_control_mode 和 trajectory_setpoint 至少以 2Hz 发布
	// 我们以 20Hz（50ms 定时器）发布以确保满足要求
	if (offboard_mode_active_ || offboard_mode_detected_) {
		// 如果检测到离板模式，确保 offboard_mode_active_ 为 true
		if (offboard_mode_detected_ && !offboard_mode_active_) {
			offboard_mode_active_ = true;
		}
		// 发布离板控制模式
		if (use_position_control_) {
			publish_offboard_control_mode(true, false, false, false, false);
		} else {
			publish_offboard_control_mode(false, true, false, false, false);
		}
		
		// 发布轨迹设定值以维持离板模式
		// 如果设定值已初始化，使用它；否则使用当前位置
		if (setpoint_initialized_) {
			if (use_position_control_) {
				publish_trajectory_setpoint(
					current_pos_setpoint_[0], 
					current_pos_setpoint_[1], 
					current_pos_setpoint_[2],
					0.0f, 0.0f, 0.0f,  // 位置控制时速度为 0
					current_yaw_setpoint_, 
					0.0f
				);
			} else {
				// 速度控制时维持当前高度
				float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
				publish_trajectory_setpoint(
					std::nanf(""), std::nanf(""), z,  // 速度控制时位置为 NaN
					current_vel_setpoint_[0],
					current_vel_setpoint_[1],
					current_vel_setpoint_[2],
					current_yaw_setpoint_,
					0.0f
				);
			}
		} else if (vehicle_local_position_received_) {
			// 使用当前位置作为设定值以保持位置
			// 用当前位置更新设定值
			current_pos_setpoint_[0] = vehicle_local_position_.x;
			current_pos_setpoint_[1] = vehicle_local_position_.y;
			current_pos_setpoint_[2] = vehicle_local_position_.z;
			current_yaw_setpoint_ = vehicle_local_position_.heading;
			setpoint_initialized_ = true;
			use_position_control_ = true;
			
			publish_offboard_control_mode(true, false, false, false, false);
			publish_trajectory_setpoint(
				vehicle_local_position_.x,
				vehicle_local_position_.y,
				vehicle_local_position_.z,
				0.0f, 0.0f, 0.0f,
				vehicle_local_position_.heading,
				0.0f
			);
		} else {
			// 尚未接收到位置，发布零设定值
			publish_offboard_control_mode(true, false, false, false, false);
			publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		
		offboard_setpoint_counter_++;
	}
}

// 进入 OFFBOARD 模式的完整握手流程：
// - 初始化设定值（使用当前位姿或 0）
// - 连续多次下发 offboard_control_mode 与 trajectory_setpoint（保证 PX4 接收到足够的设定值）
// - 发送 DO_SET_MODE 命令请求切换
void PX4ControlNode::set_mode_offboard()
{
	RCLCPP_INFO(this->get_logger(), "正在切换到离板模式...");
	
	// 如果位置可用，用当前位置初始化设定值
	if (vehicle_local_position_received_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
		current_yaw_setpoint_ = vehicle_local_position_.heading;
		setpoint_initialized_ = true;
		use_position_control_ = true;
	} else {
		current_pos_setpoint_[0] = 0.0f;
		current_pos_setpoint_[1] = 0.0f;
		current_pos_setpoint_[2] = 0.0f;
		current_yaw_setpoint_ = 0.0f;
		setpoint_initialized_ = false;
		use_position_control_ = true;
	}
	
	// 发送初始离板控制模式消息（PX4 要求）
	offboard_mode_active_ = true;
	offboard_setpoint_counter_ = 0;
	
	for (int i = 0; i < 20; ++i) {
		publish_offboard_control_mode(true, false, false, false, false);
		if (setpoint_initialized_) {
			publish_trajectory_setpoint(
				current_pos_setpoint_[0],
				current_pos_setpoint_[1],
				current_pos_setpoint_[2],
				0.0f, 0.0f, 0.0f,
				current_yaw_setpoint_,
				0.0f
			);
		} else {
			publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 6.0f, 0.0f
	);
	
	offboard_mode_active_ = true;
	offboard_mode_detected_ = true;
}

// 发布 OffboardControlMode，标识哪些控制维度被启用（位置/速度/加速度/姿态/角速率）
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

// 发布 TrajectorySetpoint：支持位置（NED）、速度、以及偏航角/偏航角速度
// 约定：当使用速度控制时，位置字段可以是 NaN；z 为高度（NED，通常为负）
void PX4ControlNode::publish_trajectory_setpoint(float x, float y, float z,
												 float vx, float vy, float vz,
												 float yaw, float yawspeed)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = get_timestamp();
	
	// 位置设定值（NED 坐标系：z 为负表示高度）
	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	
	// 速度设定值（NED 坐标系）
	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	
	// 加速度设定值（设为 NaN 表示不控制）
	msg.acceleration[0] = std::nanf("");
	msg.acceleration[1] = std::nanf("");
	msg.acceleration[2] = std::nanf("");
	
	// 偏航角和偏航角速度
	msg.yaw = yaw;
	msg.yawspeed = yawspeed;

	trajectory_setpoint_pub_->publish(msg);
}

// 直接发布位置型设定值（用于一时性控制请求）
void PX4ControlNode::publish_position_setpoint(float x, float y, float z, float yaw)
{
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	// 更新当前设定值
	current_pos_setpoint_[0] = x;
	current_pos_setpoint_[1] = y;
	current_pos_setpoint_[2] = z;
	current_yaw_setpoint_ = yaw;
	current_vel_setpoint_[0] = 0.0f;
	current_vel_setpoint_[1] = 0.0f;
	current_vel_setpoint_[2] = 0.0f;
	use_position_control_ = true;
	setpoint_initialized_ = true;
	
	// 发布启用位置控制的离板模式
	publish_offboard_control_mode(true, false, false, false, false);
	
	// 发布带位置的轨迹设定值
	publish_trajectory_setpoint(x, y, z, 0.0f, 0.0f, 0.0f, yaw, 0.0f);
}

// 直接发布速度型设定值（用于速度控制模式下的瞬时命令）
void PX4ControlNode::publish_velocity_setpoint(float vx, float vy, float vz, float yaw)
{
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	// 更新当前设定值
	current_vel_setpoint_[0] = vx;
	current_vel_setpoint_[1] = vy;
	current_vel_setpoint_[2] = vz;
	current_yaw_setpoint_ = yaw;
	use_position_control_ = false;
	setpoint_initialized_ = true;
	
	// 更新位置设定值以维持高度
	if (vehicle_local_position_received_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
	}
	
	// 发布启用速度控制的离板模式
	publish_offboard_control_mode(false, true, false, false, false);
	
	// 获取当前位置以维持（或用 NaN 表示不控制位置）
	float x = std::nanf("");
	float y = std::nanf("");
	float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
	
	// 发布带速度的轨迹设定值
	publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw, 0.0f);
}
