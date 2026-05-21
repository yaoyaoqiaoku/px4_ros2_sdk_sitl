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
#include <mutex>
#include <array>
#include <cmath>

using namespace std::chrono_literals;

class PX4ControlNode : public rclcpp::Node
{
public:
    explicit PX4ControlNode();

    // 公共控制函数
    void arm();
    void disarm();
    void arm_and_takeoff();          // 基于 ACK 的异步解锁+起飞

    // 模式切换
    void set_mode_offboard();        // 非阻塞，带超时重试
    void set_mode_land();
    void set_mode_hold();

    // 离板发布
    void publish_offboard_control_mode(bool position = true, bool velocity = false,
                                       bool acceleration = false, bool attitude = false,
                                       bool body_rate = false);
    void publish_trajectory_setpoint(float x = 0.0f, float y = 0.0f, float z = 0.0f,
                                     float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                                     float yaw = 0.0f, float yawspeed = 0.0f);
    void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);
    void publish_velocity_setpoint(float vx, float vy, float vz, float yawspeed = 0.0f);
    void publish_body_velocity_setpoint(float vx_b, float vy_b, float vz_b,
                                        float yaw = std::nanf(""),
                                        float yawspeed = 0.0f);

    // 额外飞行模式
    void set_mode_manual();
    void set_mode_altctl();
    void set_mode_posctl();
    void set_mode_acro();
    void set_mode_stabilized();
    void set_mode_posctl_orbit();
    void set_mode_auto_mission();
    void set_mode_auto_loiter();
    void set_mode_auto_rtl();
    void set_mode_auto_precland();
    void set_mode_auto_vtol_takeoff();
    void set_mode_external(uint8_t ext_index);
    void set_actuator_outputs(const std::array<float,6>& values, int index = 0);
    void set_servo_pwm(uint8_t servo_number, float pwm_usec);

private:
    // 发布器
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    // 订阅器
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_command_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr offboard_timer_;            // 离板维护 20Hz
    rclcpp::TimerBase::SharedPtr arm_timeout_timer_;         // 解锁超时定时器
    rclcpp::TimerBase::SharedPtr offboard_switch_timer_;     // OFFBOARD 切换重试定时器
    rclcpp::Time pending_setpoint_time_;                     // 外部设定值接收时间戳（超时检测）

    // 互斥锁
    mutable std::mutex state_mutex_;

    // 状态变量
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_{};
    px4_msgs::msg::VehicleStatus vehicle_status_{};
    bool vehicle_local_position_received_ = false;
    bool vehicle_status_received_ = false;
    bool offboard_mode_active_ = false;
    bool offboard_mode_detected_ = false;

    // 异步 OFFBOARD 握手
    bool offboard_handshake_active_ = false;
    int  offboard_handshake_count_ = 0;
    int  offboard_switch_attempts_ = 0;                     // 切换重试计数

    // arm_and_takeoff 异步状态
    bool takeoff_pending_ = false;    // 正等待解锁后起飞

    // 当前设定值
    float current_pos_setpoint_[3] = {0.0f, 0.0f, 0.0f};
    float current_vel_setpoint_[3] = {0.0f, 0.0f, 0.0f};
    float current_yaw_setpoint_ = 0.0f;
    float current_yawspeed_setpoint_ = 0.0f;
    bool use_position_control_ = true;
    bool setpoint_initialized_ = false;

    bool pending_external_setpoint_ = false;
    float ext_setpoint_pos_[3]   = {NAN,NAN,NAN};
    float ext_setpoint_vel_[3]   = {NAN,NAN,NAN};
    float ext_setpoint_yaw_      = NAN;
    float ext_setpoint_yawspeed_ = NAN;
    bool  ext_use_position_      = false;

    // 离板控制常量
    static constexpr int HANDSHAKE_REQUIRED = 50;          // 20Hz × 2.5s 标准握手时间
    static constexpr int MAX_OFFBOARD_SWITCH_ATTEMPTS = 10; // 最多重试10次(1秒)
    static constexpr double PENDING_SETPOINT_TIMEOUT = 5.0; // 外部设定值缓存超时

    // 回调函数
    void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg);
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
    void trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg);
    void vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg);
    void offboard_timer_callback();
    void check_offboard_switch_status();  // 检查 OFFBOARD 切换状态

    // 辅助函数
    void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
                                 float param3 = 0.0f, float param4 = 0.0f,
                                 double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);
    void set_mode_auto_takeoff();
    uint64_t get_timestamp();
    void publish_offboard_setpoint_impl();  // 需持有锁
};

#endif  // PX4_CONTROL__PX4_CONTROL_HPP_