#!/usr/bin/env python3
"""
Launch file for testing the smooth keyboard controller.

This launch file starts the smooth keyboard controller node
for testing SLAM-friendly movement commands.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for smooth keyboard controller testing."""
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start the smooth keyboard controller
    start_smooth_controller_cmd = Node(
        package='yahboom_rosmaster_system_tests',
        executable='smooth_keyboard_controller',
        name='smooth_keyboard_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add actions
    ld.add_action(start_smooth_controller_cmd)
    
    return ld 