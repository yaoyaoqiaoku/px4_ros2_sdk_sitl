#ifndef PX4_CONTROL_PX4_HOME_CONTROLLER_HPP_
#define PX4_CONTROL_PX4_HOME_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/home_position_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace px4_control
{

class PX4HomeController : public rclcpp::Node
{
public:
    explicit PX4HomeController(const std::string &node_name = "px4_home_controller_node");

    ~PX4HomeController() = default;

private:
    void home_command_callback(const px4_msgs::msg::HomePositionCommand::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void publish_home_command(const px4_msgs::msg::HomePositionCommand& cmd);
    bool check_safety_conditions();
    
    // 发布者
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // 订阅者
    rclcpp::Subscription<px4_msgs::msg::HomePositionCommand>::SharedPtr home_command_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // 状态信息
    px4_msgs::msg::VehicleStatus last_status_;
    std::mutex status_mutex_;
    
    // 配置参数
    static constexpr uint8_t PX4_TARGET_SYSTEM = 1;
    static constexpr uint8_t PX4_TARGET_COMPONENT = 1;
    static constexpr uint8_t ROS_SOURCE_SYSTEM = 1;
    static constexpr uint16_t ROS_SOURCE_COMPONENT = 1;
    static constexpr uint16_t PX4_CMD_ID = 179; // VEHICLE_CMD_DO_SET_HOME
};

} // namespace px4_control

#endif // PX4_CONTROL_PX4_HOME_CONTROLLER_HPP_