from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_dir = PathJoinSubstitution([
        FindPackageShare('target_tracker_planner'),
        'cfg'
    ])

    return LaunchDescription([
        Node(
            package='target_tracker_planner',
            executable='target_tracker_planner_node',
            name='target_tracker_planner',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'params.yaml'])
            ]
        ),
    ])