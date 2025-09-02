#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    output_bag_arg = DeclareLaunchArgument(
        'output_bag',
        default_value='lidar_recording',
        description='Output bag file name (without extension)'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/lidar/points',
        description='LiDAR topic to subscribe to'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization in RViz'
    )
    
    max_duration_arg = DeclareLaunchArgument(
        'max_duration',
        default_value='300.0',
        description='Maximum recording duration in seconds'
    )
    
    # Bag recorder node
    bag_recorder_node = Node(
        package='lidar_recorder',
        executable='bag_recorder_node',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'output_bag': LaunchConfiguration('output_bag'),
            'topic_name': LaunchConfiguration('topic_name'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'max_duration': LaunchConfiguration('max_duration'),
        }],
        remappings=[
            ('/lidar/points', LaunchConfiguration('topic_name')),
        ]
    )
    
    return LaunchDescription([
        output_bag_arg,
        topic_name_arg,
        enable_visualization_arg,
        max_duration_arg,
        bag_recorder_node,
    ])

