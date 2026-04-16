/****************************************************************************
 *
 * PX4 任务执行实现
 * 接收触发信号，通过ROS2执行航线任务，并监测任务状态
 *
 ****************************************************************************/

#include "px4_mission/px4_mission.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <std_msgs/msg/u_int16.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

PX4Mission::PX4Mission() : Node("px4_mission"), last_nav_state_(255)
{
	// 配置QoS策略
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
	
	// 任务状态发布器（JSON格式，用于MQTT桥接）
	mission_state_pub_ = this->create_publisher<std_msgs::msg::String>(
		"/px4_mission/state", 10);

	// 创建订阅器
	// 为向后兼容订阅两个话题
	mission_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/px4_mission/trigger", default_qos,
		std::bind(&PX4Mission::mission_trigger_callback, this, std::placeholders::_1));
	
	// 同时订阅旧话题以保持向后兼容性
	legacy_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/mission/trigger", default_qos,
		std::bind(&PX4Mission::mission_trigger_callback, this, std::placeholders::_1));

	vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
		"/fmu/out/vehicle_command_ack", status_qos,
		std::bind(&PX4Mission::vehicle_command_ack_callback, this, std::placeholders::_1));

	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status", status_qos,
		std::bind(&PX4Mission::vehicle_status_callback, this, std::placeholders::_1));

	// 从mavlink_mission订阅任务点总数和当前航点
	mission_count_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
		"/mission/count", default_qos,
		std::bind(&PX4Mission::mission_count_callback, this, std::placeholders::_1));
	
	current_waypoint_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
		"/mission/current_waypoint", default_qos,
		std::bind(&PX4Mission::current_waypoint_callback, this, std::placeholders::_1));

	vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		"/fmu/out/vehicle_global_position", default_qos,
		std::bind(&PX4Mission::vehicle_global_position_callback, this, std::placeholders::_1));

	// 创建控制定时器（10Hz）
	control_timer_ = this->create_wall_timer(100ms,
		std::bind(&PX4Mission::control_loop_callback, this));

	// 创建状态监控定时器（2Hz以获得更平滑的更新）
	status_timer_ = this->create_wall_timer(500ms,
		std::bind(&PX4Mission::status_monitor_callback, this));

	// 创建发布状态数据的定时器（用于MQTT桥接，2Hz）
	state_publish_timer_ = this->create_wall_timer(500ms,
		std::bind(&PX4Mission::publish_mission_state, this));

	RCLCPP_INFO(this->get_logger(), "PX4 Mission节点已启动");
}

// 任务触发回调函数
void PX4Mission::mission_trigger_callback(const std_msgs::msg::Bool::UniquePtr msg)
{
	mission_triggered_ = msg->data;
	if (mission_triggered_ && current_state_ == MissionExecState::IDLE) {
		mission_trigger_time_ = this->now();
		mission_trigger_time_set_ = true;
		last_nav_state_ = 255;
		current_state_ = MissionExecState::ARMING;
	} else if (!mission_triggered_) {
		current_state_ = MissionExecState::IDLE;
		mission_trigger_time_set_ = false;
		last_nav_state_ = 255;
	}
}

// 车辆命令应答回调函数
void PX4Mission::vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg)
{
	if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
		RCLCPP_WARN(this->get_logger(), "命令%u失败：%u", msg->command, msg->result);
	}
}

// 车辆状态回调函数
void PX4Mission::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
	if (last_nav_state_ == 255 || last_nav_state_ != msg->nav_state) {
		last_nav_state_ = msg->nav_state;
	}
	vehicle_status_ = *msg;
	vehicle_status_received_ = true;
	
	// 如果可用，从vehicle_status更新当前航点
	// 注意：PX4可能在vehicle_status中提供当前任务项索引
	// 这是一个占位符 - 实际实现取决于PX4消息结构
}

// 任务点总数回调函数
void PX4Mission::mission_count_callback(const std_msgs::msg::UInt16::UniquePtr msg)
{
	total_waypoints_ = msg->data;
	mission_count_received_ = true;
	// 任务总数已接收，无需记录
}

// 当前航点回调函数
void PX4Mission::current_waypoint_callback(const std_msgs::msg::UInt16::UniquePtr msg)
{
	uint16_t new_waypoint = msg->data;
	
	// 如果当前航点增加，更新上一个已到达航点
	if (new_waypoint > current_waypoint_) {
		if (current_waypoint_ < 65535) {
			last_reached_waypoint_ = current_waypoint_;  // 上一个航点已到达
		}
	}
	
	current_waypoint_ = new_waypoint;
	
	// 如果当前航点 >= 总航点数，任务完成
		if (mission_count_received_ && total_waypoints_ > 0 && current_waypoint_ >= total_waypoints_) {
			if (current_state_ == MissionExecState::MISSION_ACTIVE) {
				current_state_ = MissionExecState::MISSION_COMPLETE;
			}
		}
}

// 车辆全局位置回调函数
void PX4Mission::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg)
{
	vehicle_global_position_ = *msg;
	vehicle_global_position_received_ = true;

	// 在首次GPS定位时记录家位置
	if (!home_received_ && msg->lat != 0.0 && msg->lon != 0.0) {
		home_lat_ = msg->lat;
		home_lon_ = msg->lon;
		home_alt_ = msg->alt;
		home_received_ = true;
	}
}

// 控制循环回调函数
void PX4Mission::control_loop_callback()
{
	if (!mission_triggered_) {
		return;
	}

	switch (current_state_) {
		case MissionExecState::ARMING:
			handle_arming_state();
			break;
		case MissionExecState::MISSION_ACTIVE:
			handle_mission_active_state();
			break;
		case MissionExecState::MISSION_COMPLETE:
			handle_mission_complete_state();
			break;
		case MissionExecState::ERROR:
			handle_error_state();
			break;
		case MissionExecState::IDLE:
		default:
			break;
	}
}

// 状态监控回调函数
void PX4Mission::status_monitor_callback()
{
	if (!vehicle_status_received_) {
		return;
	}

	// 显示所有状态的信息（包括任务触发或等待时的空闲状态）

	// 打印头部（类似于px4_estimator格式）
	std::cout << "================================================================================\n";
	std::cout << "                            PX4任务状态                                 \n";
	std::cout << "================================================================================\n\n";
	
	// 任务状态部分
	std::cout << "[任务状态]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	const char* state_names[] = {
		"空闲", "正在解锁", "任务执行中", "任务完成", "错误"
	};
	
	std::cout << "状态: " << state_names[static_cast<int>(current_state_)] << "\n";
	
	// 车辆状态部分
	std::cout << "\n[车辆状态]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	std::string arm_status;
	if (vehicle_status_.arming_state == 2) {
		arm_status = "已解锁";
	} else if (vehicle_status_.arming_state == 1) {
		arm_status = "已锁定";
	} else {
		arm_status = "未知";
	}
	
	const char* nav_state_names[] = {
		"手动", "高度控制", "位置控制", "自动任务", "自动悬停", "自动返航", "自动降落", "自动起飞"
	};
	const char* nav_state_name = "未知";
	if (vehicle_status_.nav_state < 8) {
		nav_state_name = nav_state_names[vehicle_status_.nav_state];
	}
	
	std::cout << "锁定状态: " << arm_status << "\n";
	std::cout << "导航状态: " << static_cast<int>(vehicle_status_.nav_state) << " (" << nav_state_name << ")\n";
	
	// 任务进度部分
	std::cout << "\n[任务进度]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	if (mission_count_received_) {
		std::cout << "总航点数: " << total_waypoints_ << "\n";
		
		// 显示当前航点（为用户显示使用1起始索引）
		if (current_waypoint_ < total_waypoints_) {
			std::cout << "当前航点: " << (current_waypoint_ + 1) << " (索引 " << current_waypoint_ << ")\n";
		} else if (current_waypoint_ >= total_waypoints_) {
			std::cout << "当前航点: 已完成（所有 " << total_waypoints_ << " 个航点已到达）\n";
		} else {
			std::cout << "当前航点: " << (current_waypoint_ + 1) << "\n";
		}
		
		if (last_reached_waypoint_ != 65535) {
			std::cout << "上一个到达航点: " << (last_reached_waypoint_ + 1) << " (索引 " << last_reached_waypoint_ << ")\n";
		}
		
		if (vehicle_status_.nav_state == 3) {
			std::cout << "状态: 正在执行任务（自动任务模式）\n";
		} else if (vehicle_status_.nav_state == 4) {
			std::cout << "状态: 任务完成（自动悬停/保持模式）\n";
		}
	} else {
		std::cout << "总航点数: 等待任务总数...\n";
		if (vehicle_status_.nav_state == 3) {
			std::cout << "状态: 正在执行任务（自动任务模式）\n";
		} else if (vehicle_status_.nav_state == 4) {
			std::cout << "状态: 任务完成（自动悬停/保持模式）\n";
		}
	}
	
	// 位置部分
	if (vehicle_global_position_received_) {
		std::cout << "\n[位置]\n";
		std::cout << "--------------------------------------------------------------------------------\n";
		std::cout << std::fixed << std::setprecision(7);
		std::cout << "纬度: " << vehicle_global_position_.lat 
		          << "  经度: " << vehicle_global_position_.lon 
		          << "  高度: " << std::setprecision(2) << vehicle_global_position_.alt << "米\n";
	}
	
	std::cout << "\n";
	std::cout.flush();
}

// 处理解锁状态
void PX4Mission::handle_arming_state()
{
	static rclcpp::Time last_arm_attempt = this->now();
	static rclcpp::Time last_mode_attempt = this->now();
	const double attempt_interval = 1.0;

	if (!vehicle_status_received_ || !home_received_) {
		return;
	}

	// 如果未设置，设置任务触发时间
	if (!mission_trigger_time_set_ && mission_triggered_) {
		mission_trigger_time_ = this->now();
		mission_trigger_time_set_ = true;
	}

	// 检查任务可用性：等待3秒确保MAVLink任务已上传
	bool mission_available = false;
	if (mission_trigger_time_set_ && (this->now() - mission_trigger_time_).seconds() > 3.0) {
		mission_available = true;
	}
	
	if (!mission_available) {
		return;
	}

	// 首先设置模式为AUTO_MISSION
	if (vehicle_status_.nav_state != 3) {
		if ((this->now() - last_mode_attempt).seconds() > attempt_interval) {
			set_mode_auto_mission();
			last_mode_attempt = this->now();
		}
		return;
	}

	// 如果未解锁，解锁车辆
	if (vehicle_status_.arming_state != 2) {
		if ((this->now() - last_arm_attempt).seconds() > attempt_interval) {
			arm_vehicle();
			last_arm_attempt = this->now();
		}
		return;
	}

	// 开始任务
		start_mission();
		current_state_ = MissionExecState::MISSION_ACTIVE;
}

// 处理任务执行状态
void PX4Mission::handle_mission_active_state()
{
	if (!vehicle_status_received_) {
		return;
	}

	// 检查模式变化：nav_state 3=自动任务，4=自动悬停（保持），5=自动返航，6=自动降落
	if (vehicle_status_.nav_state != 3) {
		// nav_state 4（保持）是正常的完成状态
		if (vehicle_status_.nav_state == 4 || vehicle_status_.nav_state == 5 || 
		    vehicle_status_.nav_state == 6 || vehicle_status_.arming_state != 2) {
			current_state_ = MissionExecState::MISSION_COMPLETE;
		}
	}
}

// 处理任务完成状态
void PX4Mission::handle_mission_complete_state()
{
	// 任务完成，返回空闲状态（无自动返航）
	// 用户可以在任务中设置返航点作为最后一个航点
	current_state_ = MissionExecState::IDLE;
	mission_triggered_ = false;
	last_reached_waypoint_ = 65535;
}

// 处理错误状态
void PX4Mission::handle_error_state()
{
	current_state_ = MissionExecState::IDLE;
	mission_triggered_ = false;
}

// 获取时间戳
uint64_t PX4Mission::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

// 发布车辆命令
void PX4Mission::publish_vehicle_command(uint32_t command, float param1, float param2,
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

// 设置自动任务模式
void PX4Mission::set_mode_auto_mission()
{
	// VEHICLE_CMD_DO_SET_MODE: param1=1, param2=4（自动）, param3=4（自动任务）
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 4.0f);
}

// 解锁车辆
void PX4Mission::arm_vehicle()
{
	if (vehicle_status_.arming_state != 2) {
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	}
}

// 开始任务
void PX4Mission::start_mission()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_MISSION_START, 0.0f, 0.0f);
}

// 发布任务状态
void PX4Mission::publish_mission_state()
{
	json j;
	
	// 任务状态
	const char* state_names[] = {
		"空闲", "正在解锁", "任务执行中", "任务完成", "错误"
	};
	j["state"] = state_names[static_cast<int>(current_state_)];
	j["mission_triggered"] = mission_triggered_;
	
	// 车辆状态
	if (vehicle_status_received_) {
		j["armed"] = (vehicle_status_.arming_state == 2);
		j["arming_state"] = static_cast<int>(vehicle_status_.arming_state);
		j["nav_state"] = static_cast<int>(vehicle_status_.nav_state);
	}
	
	// 任务进度
	j["total_waypoints"] = total_waypoints_;
	j["current_waypoint"] = current_waypoint_;
	if (last_reached_waypoint_ != 65535) {
		j["last_reached_waypoint"] = last_reached_waypoint_;
	}
	j["mission_count_received"] = mission_count_received_;
	
	// 位置
	if (vehicle_global_position_received_) {
		j["position"] = {
			{"lat", vehicle_global_position_.lat},
			{"lon", vehicle_global_position_.lon},
			{"alt", vehicle_global_position_.alt}
		};
	}
	
	// 时间戳
	j["timestamp"] = this->now().nanoseconds();
	
	// 发布为字符串消息
	auto msg = std_msgs::msg::String();
	msg.data = j.dump();
	mission_state_pub_->publish(msg);
}