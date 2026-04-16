#!/bin/bash
set -euo pipefail  # 严格的错误处理：遇到错误退出、未定义变量报错、管道错误检测

# 脚本名称: compile_control.sh
# 脚本描述: 编译PX4 ROS2控制模块（适配新增px4_arucoland等模块）

# 颜色定义（增强输出可读性）
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 主函数
main() {
    echo "=========================================="
    echo "开始编译PX4 ROS2控制模块（完整版）"
    echo "=========================================="
    echo ""

    # 进入工作空间目录
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    print_info "工作空间路径: $SCRIPT_DIR"
    cd "$SCRIPT_DIR"

    # 检查ROS2环境
    if [ -z "${ROS_DISTRO:-}" ]; then
        print_warn "ROS2环境未设置，尝试自动加载..."
        local ros_setup_files=(
            "/opt/ros/humble/setup.bash"
            "/opt/ros/foxy/setup.bash"
            "/opt/ros/iron/setup.bash"
            "/opt/ros/jazzy/setup.bash"
        )
        
        local ros_setup_loaded=false
        for setup_file in "${ros_setup_files[@]}"; do
            if [ -f "$setup_file" ]; then
                source "$setup_file"
                print_info "成功加载ROS2环境: $setup_file"
                ros_setup_loaded=true
                break
            fi
        done
        
        if [ "$ros_setup_loaded" = false ]; then
            print_error "未找到ROS2安装，请先安装ROS2或手动source环境"
            exit 1
        fi
    else
        print_info "已加载ROS2环境: $ROS_DISTRO"
    fi

    # 编译函数（封装重复逻辑）
    compile_package() {
        local pkg_name=$1
        local pkg_desc=$2
        local is_optional=${3:-false}
        
        echo ""
        print_info "开始编译 $pkg_desc ($pkg_name)..."
        
        if colcon build --packages-select "$pkg_name" --event-handlers console_direct+; then
            print_info "$pkg_desc 编译成功"
        else
            if [ "$is_optional" = true ]; then
                print_warn "$pkg_desc 编译失败（可选包，继续执行）"
            else
                print_error "$pkg_desc 编译失败，退出脚本"
                exit 1
            fi
        fi
    }

    # 编译顺序（按依赖关系排序）
    # 1. microxrcedds_agent (基础依赖)
    compile_package "microxrcedds_agent" "Micro-XRCE-DDS-Agent"
    
    # 2. px4_msgs (消息定义)
    compile_package "px4_msgs" "PX4消息定义"
    
    # 3. px4_ros_com (ROS2通信库)
    compile_package "px4_ros_com" "PX4 ROS2通信库"
    
    # 4. px4-ros2-interface-lib (PX4 ROS2接口库)
    compile_package "px4_ros2_cpp" "PX4 ROS2接口库"
    
    # 5. ARUCO定位与精准着陆模块
    compile_package "aruco_tracker" "ARUCO标记追踪模块"
    compile_package "precision_land" "精准着陆控制模块"
    
    # 6. px4_control (核心控制模块)
    compile_package "px4_control" "PX4核心控制模块"
    
    # 7. px4_mqtt (MQTT桥接模块，可选)
    compile_package "px4_mqtt" "MQTT桥接模块" true


    echo ""
    echo "=========================================="
    print_info "所有必选模块编译完成！"
    echo "=========================================="

    # 清理log文件（可选）
    if [ -d "log" ]; then
        echo ""
        print_info "清理编译日志文件..."
        rm -rf log/*
        print_info "日志文件已清理"
    fi

    # 环境设置提示
    echo ""
    print_info "请运行以下命令设置环境："
    echo "  source $SCRIPT_DIR/install/setup.bash"
    echo ""
    print_info "常用启动命令参考："
    echo "  # 启动PX4控制节点"
    echo "  ros2 launch px4_control px4_control.launch.py"
    echo "  # 启动ARUCO精准着陆"
    echo "  ros2 launch px4_arucoland precision_landing_system.launch.py"
    echo ""
}

# 执行主函数
main "$@"

