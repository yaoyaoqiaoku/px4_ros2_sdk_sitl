#include "rclcpp/rclcpp.hpp"
#include "px4_control/px4_servo_controller.hpp"

int main(int argc, char *argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建并运行舵机控制节点（保持原有节点名称，无需修改启动脚本）
    auto servo_controller_node = std::make_shared<px4_control::PX4ServoController>();
    rclcpp::spin(servo_controller_node);

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
