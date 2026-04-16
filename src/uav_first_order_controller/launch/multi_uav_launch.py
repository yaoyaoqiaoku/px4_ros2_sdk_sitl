import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # 控制器节点列表（uav1~6）
    uav_ids = [1, 2, 3, 4, 5, 6]
    
    for uav_id in uav_ids:
        # 创建每个无人机的控制器节点
        controller_node = Node(
            package="uav_first_order_controller",
            executable=f"uav{uav_id}_controller",
            output="screen",
            name=f"uav{uav_id}_controller",
            parameters=[{
                "use_sim_time": True  # 使用仿真时间（匹配PX4/GZ）
            }],
            # 可选：输出日志到文件（方便调试）
            arguments=['--ros-args', '--log-level', 'INFO']
        )
        ld.add_action(controller_node)

    return ld
