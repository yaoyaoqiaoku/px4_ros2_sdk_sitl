#ifndef PX4_CONTROL_PX4_SERVO_CONTROLLER_HPP_
#define PX4_CONTROL_PX4_SERVO_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/servo_command.hpp"
#include <array>

namespace px4_control
{

class PX4ServoController : public rclcpp::Node
{
public:
    explicit PX4ServoController(const std::string &node_name = "px4_servo_controller_node");

    ~PX4ServoController() = default;

private:
    void declare_parameters();
    void update_parameters();
    void publish_actuator_command();
    
    // 新增：处理外部舵机命令
    void servo_command_callback(const px4_msgs::msg::ServoCommand::SharedPtr msg);
    void apply_servo_command(const px4_msgs::msg::ServoCommand& cmd);

    // 发布者
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // 新增：订阅者 - 接收外部舵机命令
    rclcpp::Subscription<px4_msgs::msg::ServoCommand>::SharedPtr servo_command_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr command_timer_;

    // 舵机控制参数
    std::array<float, 6> actuators_;  // 使用数组存储6路舵机值
    int actuator_index_;
    float send_frequency_;
    bool external_command_received_;  // 标记是否收到外部命令

    // PX4通信配置
    static constexpr uint8_t PX4_TARGET_SYSTEM = 1;
    static constexpr uint8_t PX4_TARGET_COMPONENT = 1;
    static constexpr uint8_t ROS_SOURCE_SYSTEM = 1;
    static constexpr uint16_t ROS_SOURCE_COMPONENT = 1;
    static constexpr uint16_t PX4_CMD_ID = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;
};

} // namespace px4_control

#endif // PX4_CONTROL_PX4_SERVO_CONTROLLER_HPP_