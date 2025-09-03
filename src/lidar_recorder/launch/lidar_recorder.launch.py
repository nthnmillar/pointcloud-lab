#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = os.path.join(os.getcwd(), 'src/lidar_recorder')
    
    # Launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='recorded_lidar_data',
        description='Output directory for recorded LiDAR data'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/lidar/points',
        description='LiDAR topic to subscribe to'
    )
    
    raw_arg = DeclareLaunchArgument(
        'raw',
        default_value='true',
        description='Whether to save raw (unfiltered) data'
    )
    
    filtered_arg = DeclareLaunchArgument(
        'filtered',
        default_value='true',
        description='Whether to apply filtering and save filtered data'
    )
    
    filter_lvl_arg = DeclareLaunchArgument(
        'filter_lvl',
        default_value='0.1',
        description='Filter level (voxel size, 0.0 to disable)'
    )
    
    save_individual_files_arg = DeclareLaunchArgument(
        'save_individual_files',
        default_value='false',
        description='Whether to save individual PCD files for each message'
    )
    
    output_format_arg = DeclareLaunchArgument(
        'output_format',
        default_value='ply',
        description='Output format: ply or pcd'
    )
    
    stop_recording_arg = DeclareLaunchArgument(
        'stop_recording',
        default_value='false',
        description='Set to true to stop recording and save data'
    )
    
    # LiDAR recorder node
    lidar_recorder_node = Node(
        package='lidar_recorder',
        executable='lidar_recorder_node',
        name='lidar_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'topic_name': LaunchConfiguration('topic_name'),
            'raw': LaunchConfiguration('raw'),
            'filtered': LaunchConfiguration('filtered'),
            'filter_lvl': LaunchConfiguration('filter_lvl'),
            'save_individual_files': LaunchConfiguration('save_individual_files'),
            'output_format': LaunchConfiguration('output_format'),
            'stop_recording': LaunchConfiguration('stop_recording'),
        }],
        remappings=[
            ('/lidar/points', LaunchConfiguration('topic_name')),
        ]
    )
    
    return LaunchDescription([
        output_dir_arg,
        topic_name_arg,
        raw_arg,
        filtered_arg,
        filter_lvl_arg,
        save_individual_files_arg,
        output_format_arg,
        stop_recording_arg,
        lidar_recorder_node,
    ])

