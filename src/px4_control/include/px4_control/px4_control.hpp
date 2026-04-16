/****************************************************************************
 *
 * PX4 控制节点
 * 提供 PX4 的控制接口：解锁/上锁、解锁+起飞和离板控制
 *
 ****************************************************************************/
// --- 中文说明 -----------------------------------------------------------
// PX4ControlNode 是一个对 PX4 飞控进行控制的 ROS2 节点，提供一组便捷的
// 控制接口，用于：
// - 解/上锁（arm/disarm）
// - 一键解锁并起飞（arm_and_takeoff）
// - 切换到 OFFBOARD 模式并通过定时器持续下发设定值（位置/速度）
// - 将外部 VehicleCommand 转发至 PX4（通过 /fmu/in/vehicle_command）
//
// 设计要点：
// - 使用合适的 QoS 以匹配 PX4 的发布策略（例如 BestEffort + TransientLocal）
// - OFFBOARD 模式需要以至少 2Hz 的频率下发 offboard_control_mode 与 trajectory_setpoint
// - 节点内部维护当前设定值（位置或速度）并由定时器持续发布以维持离板模式
//
// 注意：很多函数直接构造并发布 px4_msgs 消息，调用者需要对坐标系（NED）和单位有正确理解
// ------------------------------------------------------------------------

#ifndef PX4_CONTROL__PX4_CONTROL_HPP_
#define PX4_CONTROL__PX4_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class PX4ControlNode : public rclcpp::Node
{
public:
	explicit PX4ControlNode();

	// 公共控制函数
	void arm();                     // 解锁
	void disarm();                  // 上锁
	void arm_and_takeoff();         // 解锁然后切换到自动起飞模式
	
	// 模式切换函数
	void set_mode_offboard();       // 切换到离板模式
	void set_mode_land();           // 切换到自动着陆模式
	void set_mode_hold();           // 切换到自动悬停模式
	
	// 离板模式函数
	// 发布离板控制模式（指定哪些控制维度有效）
	void publish_offboard_control_mode(bool position = true, bool velocity = false, 
	                                   bool acceleration = false, bool attitude = false, 
	                                   bool body_rate = false);
	// 发布轨迹设定值（位置/速度等）
	void publish_trajectory_setpoint(float x = 0.0f, float y = 0.0f, float z = 0.0f, 
	                                 float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
	                                 float yaw = 0.0f, float yawspeed = 0.0f);
	// 发布位置设定值
	void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);
	// 发布速度设定值
	void publish_velocity_setpoint(float vx, float vy, float vz, float yaw = 0.0f);

	// 额外的模式切换与常用命令补充
	void set_mode_manual();
	void set_mode_altctl();
	void set_mode_posctl();
	void set_mode_acro();
	void set_mode_stabilized();
	void set_mode_posctl_orbit();

	// 自动模式的子模式
	void set_mode_auto_mission();
	void set_mode_auto_loiter();
	void set_mode_auto_rtl();
	void set_mode_auto_precland();
	void set_mode_auto_vtol_takeoff();

	// External sub-modes (EXTERNAL1..EXTERNAL8)
	void set_mode_external(uint8_t ext_index); // 1..8

	// Actuator / Servo commands
	void set_actuator_outputs(const std::array<float,6>& values, int index = 0);
	void set_servo_pwm(uint8_t servo_number, float pwm_usec);

private:
	// 发布器
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;          // 飞行器命令发布器
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;  // 离板控制模式发布器
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;    // 轨迹设定值发布器

	// 订阅器
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;      // 命令应答订阅器
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;  // 本地位置订阅器
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;                // 飞行器状态订阅器
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_command_sub_;  // 轨迹设定值命令订阅器
	rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_sub_;              // 飞行器命令订阅器

	// 离板控制模式周期发布定时器（至少 2Hz）
	rclcpp::TimerBase::SharedPtr offboard_timer_;

	// 状态变量
	px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;  // 飞行器本地位置
	px4_msgs::msg::VehicleStatus vehicle_status_;                  // 飞行器状态
	bool vehicle_local_position_received_ = false;                 // 是否接收到本地位置
	bool vehicle_status_received_ = false;                         // 是否接收到飞行器状态
	bool offboard_mode_active_ = false;                            // 离板模式是否激活
	bool offboard_mode_detected_ = false;                          // 从飞行器状态检测到离板模式
	uint64_t offboard_setpoint_counter_ = 0;                       // 离板设定值计数器
	
	// 当前设定值（用于维持离板模式）
	float current_pos_setpoint_[3] = {0.0f, 0.0f, 0.0f};  // 当前位置设定值
	float current_vel_setpoint_[3] = {0.0f, 0.0f, 0.0f};  // 当前速度设定值
	float current_yaw_setpoint_ = 0.0f;                   // 当前偏航角设定值
	bool use_position_control_ = true;                    // 控制模式：true=位置控制，false=速度控制
	bool setpoint_initialized_ = false;                   // 设定值是否已初始化

	// 回调函数
	void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg);      // 飞行器命令应答回调
	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);  // 本地位置回调
	void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg);                // 飞行器状态回调
	void trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg);  // 轨迹设定值命令回调
	void vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg);              // 飞行器命令回调
	void offboard_timer_callback();  // 离板模式定时器回调

	// 辅助函数
	// 发布飞行器命令
	void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
	                             float param3 = 0.0f, float param4 = 0.0f, 
	                             double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);
	void set_mode_auto_takeoff();  // 切换到自动起飞模式
	uint64_t get_timestamp();      // 获取时间戳
};

#endif  // PX4_CONTROL__PX4_CONTROL_HPP_
