#include "px4_control/px4_servo_controller.hpp"
#include <algorithm>

namespace px4_control
{

PX4ServoController::PX4ServoController(const std::string &node_name)
    : Node(node_name),
      actuators_({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      actuator_index_(0),
      send_frequency_(10.0f),
      external_command_received_(false)
{
    // 1. 声明并加载参数
    declare_parameters();
    update_parameters();

    // 2. 创建发布者
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", rclcpp::QoS(10));

    // 3. 创建订阅者 - 接收外部舵机命令
    servo_command_sub_ = this->create_subscription<px4_msgs::msg::ServoCommand>(
        "/px4_servo/command",
        rclcpp::QoS(10),
        std::bind(&PX4ServoController::servo_command_callback, this, std::placeholders::_1));

    // 4. 创建定时器，按配置频率发布命令
    auto publish_period = std::chrono::duration<float>(1.0f / send_frequency_);
    command_timer_ = this->create_wall_timer(
        publish_period, std::bind(&PX4ServoController::publish_actuator_command, this));

    // 5. 输出初始化日志
    RCLCPP_INFO(this->get_logger(), "[PX4 Servo Controller] Initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribe Topic: /px4_servo/command");
    RCLCPP_INFO(this->get_logger(), "Publish Topic: /fmu/in/vehicle_command");
    RCLCPP_INFO(this->get_logger(), "Initial Config - Actuators: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                actuators_[0], actuators_[1], actuators_[2], 
                actuators_[3], actuators_[4], actuators_[5]);
    RCLCPP_INFO(this->get_logger(), "Actuator Index: %d, Publish Frequency: %.1fHz",
                actuator_index_, send_frequency_);
}

void PX4ServoController::declare_parameters()
{
    // 声明默认参数值（当没有外部命令时使用）
    this->declare_parameter<float>("actuator_1", 0.0f);
    this->declare_parameter<float>("actuator_2", 0.0f);
    this->declare_parameter<float>("actuator_3", 0.0f);
    this->declare_parameter<float>("actuator_4", 0.0f);
    this->declare_parameter<float>("actuator_5", 0.0f);
    this->declare_parameter<float>("actuator_6", 0.0f);
    this->declare_parameter<int>("actuator_index", 0);
    this->declare_parameter<float>("send_frequency", 10.0f);
}

void PX4ServoController::update_parameters()
{
    // 仅在没有外部命令时使用参数
    if (!external_command_received_) {
        this->get_parameter("actuator_1", actuators_[0]);
        this->get_parameter("actuator_2", actuators_[1]);
        this->get_parameter("actuator_3", actuators_[2]);
        this->get_parameter("actuator_4", actuators_[3]);
        this->get_parameter("actuator_5", actuators_[4]);
        this->get_parameter("actuator_6", actuators_[5]);
        this->get_parameter("actuator_index", actuator_index_);
        this->get_parameter("send_frequency", send_frequency_);
    }

    // 限制参数范围
    for (auto& actuator : actuators_) {
        actuator = std::clamp(actuator, -1.0f, 1.0f);
    }
    actuator_index_ = std::clamp(actuator_index_, 0, 1);
    send_frequency_ = std::clamp(send_frequency_, 1.0f, 100.0f);
}

void PX4ServoController::servo_command_callback(const px4_msgs::msg::ServoCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received servo command via topic");
    apply_servo_command(*msg);
}

void PX4ServoController::apply_servo_command(const px4_msgs::msg::ServoCommand& cmd)
{
    // 更新舵机值
    for (size_t i = 0; i < 6 && i < cmd.actuators.size(); ++i) {
        actuators_[i] = std::clamp(cmd.actuators[i], -1.0f, 1.0f);
    }
    
    // 更新舵机组索引
    actuator_index_ = std::clamp(static_cast<int>(cmd.actuator_index), 0, 1);
    
    // 更新发布频率
    if (cmd.send_frequency > 0) {
        send_frequency_ = std::clamp(cmd.send_frequency, 1.0f, 100.0f);
        
        // 重新配置定时器
        auto publish_period = std::chrono::duration<float>(1.0f / send_frequency_);
        command_timer_->cancel();
        command_timer_ = this->create_wall_timer(
            publish_period, std::bind(&PX4ServoController::publish_actuator_command, this));
    }
    
    external_command_received_ = true;
    
    // 如果需要立即更新，则立即发布一次
    if (cmd.update) {
        publish_actuator_command();
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Servo command applied - Actuators: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Index: %d, Freq: %.1fHz",
                actuators_[0], actuators_[1], actuators_[2],
                actuators_[3], actuators_[4], actuators_[5],
                actuator_index_, send_frequency_);
}

void PX4ServoController::publish_actuator_command()
{
    // 构造PX4 VehicleCommand消息
    px4_msgs::msg::VehicleCommand cmd_msg;

    // 1. 时间戳
    cmd_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // 2. 命令ID：VEHICLE_CMD_DO_SET_ACTUATOR（固定187）
    cmd_msg.command = PX4_CMD_ID;

    // 3. 舵机输出值（param1-param6对应6路舵机）
    cmd_msg.param1 = actuators_[0];
    cmd_msg.param2 = actuators_[1];
    cmd_msg.param3 = actuators_[2];
    cmd_msg.param4 = actuators_[3];
    cmd_msg.param5 = actuators_[4];
    cmd_msg.param6 = actuators_[5];

    // 4. 舵机组索引（param7）
    cmd_msg.param7 = static_cast<float>(actuator_index_);

    // 5. 目标系统/组件
    cmd_msg.target_system = PX4_TARGET_SYSTEM;
    cmd_msg.target_component = PX4_TARGET_COMPONENT;

    // 6. 源系统/组件
    cmd_msg.source_system = ROS_SOURCE_SYSTEM;
    cmd_msg.source_component = ROS_SOURCE_COMPONENT;

    // 7. 确认标志
    cmd_msg.confirmation = 0;

    // 8. 外部命令标志
    cmd_msg.from_external = true;

    // 发布消息到PX4
    vehicle_command_pub_->publish(cmd_msg);

    RCLCPP_DEBUG(this->get_logger(), 
                 "Published Servo Command - [A1:%.2f, A2:%.2f, A3:%.2f, A4:%.2f, A5:%.2f, A6:%.2f], Index:%d",
                 actuators_[0], actuators_[1], actuators_[2],
                 actuators_[3], actuators_[4], actuators_[5],
                 actuator_index_);
}

} // namespace px4_control