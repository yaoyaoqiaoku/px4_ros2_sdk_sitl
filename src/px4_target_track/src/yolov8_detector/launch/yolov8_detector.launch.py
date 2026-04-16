from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 参数声明
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8n',
        description='YOLOv8 model size (nano, small, medium, large, xlarge)'
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Detection confidence threshold'
    )

    # 获取配置文件
    config_dir = PathJoinSubstitution([
        FindPackageShare('yolov8_detector'),
        'cfg'
    ])

    return LaunchDescription([
        model_arg,
        confidence_arg,

        Node(
            package='yolov8_detector',
            executable='yolov8_detector_node',
            name='yolov8_detector',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'params.yaml']),
                {
                    'model_path': LaunchConfiguration('model'),
                    'confidence_threshold': LaunchConfiguration('confidence'),
                }
            ]
        ),
    ])