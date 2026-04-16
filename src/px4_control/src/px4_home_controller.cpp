#include "px4_control/px4_home_controller.hpp"
#include <algorithm>
#include <mutex>

namespace px4_control
{

PX4HomeController::PX4HomeController(const std::string &node_name)
    : Node(node_name)
{
    // 创建发布者（显式指定 reliable 策略，贴合 PX4 /fmu/in/ 系列话题规范）
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        rclcpp::QoS(10).reliable());

    // 定义 PX4 标准 QoS 配置：尽力而为（best_effort）+ 保留 10 条历史消息
    // 匹配 PX4 /fmu/out/ 系列高频状态话题的 QoS 策略，解决兼容警告
    rclcpp::QoS px4_qos = rclcpp::QoS(10).best_effort();

    // 创建订阅者（/px4_home/command - 自定义话题，使用 PX4 兼容 QoS）
    home_command_sub_ = this->create_subscription<px4_msgs::msg::HomePositionCommand>(
        "/px4_home/command",
        px4_qos,
        std::bind(&PX4HomeController::home_command_callback, this, std::placeholders::_1));
        
    // 创建订阅者（/fmu/out/vehicle_status - PX4 状态话题，使用 best_effort QoS 解决兼容问题）
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        px4_qos,
        std::bind(&PX4HomeController::vehicle_status_callback, this, std::placeholders::_1));

    // 输出初始化日志
    RCLCPP_INFO(this->get_logger(), "[PX4 Home Controller] Initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "Subscribe Topic: /px4_home/command");
    RCLCPP_INFO(this->get_logger(), "Subscribe Topic: /fmu/out/vehicle_status (QoS: best_effort)");
    RCLCPP_INFO(this->get_logger(), "Publish Topic: /fmu/in/vehicle_command (QoS: reliable)");
}

void PX4HomeController::home_command_callback(const px4_msgs::msg::HomePositionCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received home position command");
    
    // 安全检查
    if (!check_safety_conditions()) {
        RCLCPP_WARN(this->get_logger(), "Safety check failed, ignoring home command");
        return;
    }
    
    // 处理Home位置命令
    publish_home_command(*msg);
}

void PX4HomeController::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_ = *msg;
}

void PX4HomeController::publish_home_command(const px4_msgs::msg::HomePositionCommand& cmd)
{
    // 构造PX4 VehicleCommand消息
    px4_msgs::msg::VehicleCommand vehicle_cmd;

    // 1. 时间戳
    vehicle_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // 2. 命令ID：VEHICLE_CMD_DO_SET_HOME（179）
    vehicle_cmd.command = PX4_CMD_ID;

    // 3. 参数设置
    if (cmd.use_current) {
        // 使用当前位置
        vehicle_cmd.param1 = 1.0f;  // 使用当前位置
        vehicle_cmd.param5 = 0.0;    // 纬度（不使用）
        vehicle_cmd.param6 = 0.0;    // 经度（不使用）
        vehicle_cmd.param7 = 0.0f;   // 海拔（不使用）
    } else {
        // 使用指定坐标
        vehicle_cmd.param1 = 0.0f;  // 使用指定坐标
        vehicle_cmd.param5 = cmd.lat; // 纬度
        vehicle_cmd.param6 = cmd.lon; // 经度
        vehicle_cmd.param7 = cmd.alt; // 海拔
    }

    // 4. 目标系统/组件
    vehicle_cmd.target_system = PX4_TARGET_SYSTEM;
    vehicle_cmd.target_component = PX4_TARGET_COMPONENT;

    // 5. 源系统/组件
    vehicle_cmd.source_system = ROS_SOURCE_SYSTEM;
    vehicle_cmd.source_component = ROS_SOURCE_COMPONENT;

    // 6. 确认标志
    vehicle_cmd.confirmation = 0;

    // 7. 外部命令标志
    vehicle_cmd.from_external = true;

    // 发布消息到PX4
    vehicle_command_pub_->publish(vehicle_cmd);

    if (cmd.use_current) {
        RCLCPP_INFO(this->get_logger(), "Published Home Command: Use current position");
    } else {
        RCLCPP_INFO(this->get_logger(), 
                   "Published Home Command: lat=%.6f, lon=%.6f, alt=%.1f", 
                   cmd.lat, cmd.lon, cmd.alt);
    }
}

bool PX4HomeController::check_safety_conditions()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 检查飞行器是否已解锁
    if (last_status_.arming_state == 2) { // 2 = ARMED（已解锁状态）
        RCLCPP_WARN(this->get_logger(), "Vehicle is armed, home setting may be unsafe");
        // 如需严格禁止解锁状态下设置家点，取消下面注释即可
        // return false;
    }
    
    // 检查GPS状态（扩展：可通过 last_status_.gps_status 判断GPS是否有效）
    // if (last_status_.gps_status < 3) { // 3 通常表示GPS固定（3D Fix）
    //     RCLCPP_WARN(this->get_logger(), "GPS not fixed, home setting may be invalid");
    //     // return false;
    // }
    
    return true; // 暂时允许所有情况
}

} // namespace px4_control

// 主函数
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_control::PX4HomeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
