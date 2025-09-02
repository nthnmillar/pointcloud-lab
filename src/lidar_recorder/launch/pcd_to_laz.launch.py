#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    input_pcd_arg = DeclareLaunchArgument(
        'input_pcd',
        default_value='',
        description='Input PCD file path (required)'
    )
    
    output_laz_arg = DeclareLaunchArgument(
        'output_laz',
        default_value='',
        description='Output LAZ file path (optional, auto-generated if empty)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable PCL visualization'
    )
    
    compression_level_arg = DeclareLaunchArgument(
        'compression_level',
        default_value='3',
        description='LAZ compression level (0-9, higher = more compression)'
    )
    
    point_format_arg = DeclareLaunchArgument(
        'point_format',
        default_value='0',
        description='Point format: 0=XYZ, 1=XYZI, 2=XYZRGB'
    )
    
    # PCD to LAZ converter node
    pcd_to_laz_node = Node(
        package='lidar_recorder',
        executable='pcd_to_laz_converter',
        name='pcd_to_laz_converter',
        output='screen',
        parameters=[{
            'input_pcd': LaunchConfiguration('input_pcd'),
            'output_laz': LaunchConfiguration('output_laz'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'compression_level': LaunchConfiguration('compression_level'),
            'point_format': LaunchConfiguration('point_format'),
        }]
    )
    
    return LaunchDescription([
        input_pcd_arg,
        output_laz_arg,
        enable_visualization_arg,
        compression_level_arg,
        point_format_arg,
        pcd_to_laz_node,
    ])

