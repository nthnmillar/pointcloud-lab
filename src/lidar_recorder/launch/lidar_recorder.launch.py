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
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.1',
        description='Voxel grid size for filtering (0.0 to disable)'
    )
    
    remove_outliers_arg = DeclareLaunchArgument(
        'remove_outliers',
        default_value='true',
        description='Whether to remove statistical outliers'
    )
    
    outlier_std_dev_arg = DeclareLaunchArgument(
        'outlier_std_dev',
        default_value='2.0',
        description='Standard deviation multiplier for outlier removal'
    )
    
    outlier_min_neighbors_arg = DeclareLaunchArgument(
        'outlier_min_neighbors',
        default_value='10',
        description='Minimum neighbors for outlier removal'
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
    
    # LiDAR recorder node
    lidar_recorder_node = Node(
        package='lidar_recorder',
        executable='lidar_recorder_node',
        name='lidar_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'topic_name': LaunchConfiguration('topic_name'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'remove_outliers': LaunchConfiguration('remove_outliers'),
            'outlier_std_dev': LaunchConfiguration('outlier_std_dev'),
            'outlier_min_neighbors': LaunchConfiguration('outlier_min_neighbors'),
            'save_individual_files': LaunchConfiguration('save_individual_files'),
            'output_format': LaunchConfiguration('output_format'),
        }],
        remappings=[
            ('/lidar/points', LaunchConfiguration('topic_name')),
        ]
    )
    
    return LaunchDescription([
        output_dir_arg,
        topic_name_arg,
        voxel_size_arg,
        remove_outliers_arg,
        outlier_std_dev_arg,
        outlier_min_neighbors_arg,
        save_individual_files_arg,
        output_format_arg,
        lidar_recorder_node,
    ])

