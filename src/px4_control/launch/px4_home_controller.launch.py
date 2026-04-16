#!/usr/bin/env python3

"""
Launch file for PX4 Home Controller node.

Starts the `px4_home_controller` executable from the `px4_control` package.
This launch follows the project's existing conventions: output to screen and
an optional parameters list (add defaults below if you have a params YAML).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='px4_control',
			executable='px4_home_controller',
			name='px4_home_controller',
			output='screen',
			parameters=[
				# Add dicts or YAML file paths here if you want to load parameters by default.
				# Example: str(Path(get_package_share_directory('px4_control')) / 'config' / 'px4_home_controller.yaml')
			]
		)
	])

