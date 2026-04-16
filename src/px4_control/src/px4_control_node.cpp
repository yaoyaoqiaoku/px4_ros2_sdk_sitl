/****************************************************************************
 *
 * PX4 Control Node Main Entry Point
 *
 ****************************************************************************/
// --- 中文说明 -----------------------------------------------------------
// 进程入口：初始化 ROS2，创建 PX4ControlNode 实例并启动 spin 循环。
// 该文件仅包含 main()，不包含业务逻辑，便于在容器/系统服务中简单启动节点。
// ------------------------------------------------------------------------

#include "px4_control/px4_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PX4ControlNode>();
	rclcpp::spin(node);
	
	rclcpp::shutdown();
	return 0;
}

