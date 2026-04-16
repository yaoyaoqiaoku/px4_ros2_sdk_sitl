from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    enable_viz_arg = DeclareLaunchArgument(
        'enable_gazebo_viz',
        default_value='true',
        description='Enable Gazebo marker visualization (simulation only)'
    )
    
    # 3. 构建启动描述
    return LaunchDescription([
        # 参数声明
        model_arg,
        confidence_arg,
        enable_viz_arg,

        # ========== Gazebo桥接节点 ==========
        # 桥接Gazebo的/camera到ROS2的/camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen',
        ),

        # 桥接Gazebo的/camera_info到ROS2的/camera_info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[
                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
        ),

        # 桥接处理后的图像
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_proc_bridge',
            arguments=[
                '/image_proc@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            parameters=[{
                'qos_overrides./image_proc.subscription.reliability': 'best_effort',
                'qos_overrides./image_proc.publisher.reliability': 'best_effort'
            }],
            output='screen',
        ),

        # ========== YOLOv8检测节点 ==========
        Node(
            package='yolov8_detector',
            executable='yolov8_detector_node',
            name='yolov8_detector_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('yolov8_detector'), 'cfg', 'params.yaml']),
            ]
        ),

        # ========== 目标跟踪规划节点 ==========
        Node(
            package='target_tracker_planner',
            executable='target_tracker_planner_node',
            name='target_tracker_planner',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('target_tracker_planner'), 'cfg', 'params.yaml'])
            ]
        ),
    ])
