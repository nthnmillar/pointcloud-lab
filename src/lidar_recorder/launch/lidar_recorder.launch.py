#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def get_format_enabled(config_name, context):
    """Helper function to check if a format is enabled"""
    return LaunchConfiguration(config_name).perform(context) == 'true'

def get_laz_enabled(context):
    """Determine if LAZ should be enabled based on other format settings"""
    laz_explicit = get_format_enabled('laz', context)
    pcd_enabled = get_format_enabled('pcd', context)
    ply_enabled = get_format_enabled('ply', context)
    las_enabled = get_format_enabled('las', context)
    
    # LAZ is enabled if:
    # 1. Explicitly set to true, OR
    # 2. No other formats are explicitly enabled (default behavior)
    return laz_explicit or (not pcd_enabled and not ply_enabled and not las_enabled)

def create_lidar_recorder_node(context):
    """Create the LiDAR recorder node with computed parameters"""
    # Determine format settings
    pcd_enabled = get_format_enabled('pcd', context)
    ply_enabled = get_format_enabled('ply', context)
    las_enabled = get_format_enabled('las', context)
    laz_enabled = get_laz_enabled(context)
    
    return [Node(
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
            'pcd': pcd_enabled,
            'ply': ply_enabled,
            'laz': laz_enabled,
            'las': las_enabled,
            'stop_recording': LaunchConfiguration('stop_recording'),
        }],
        remappings=[
            ('/lidar/points', LaunchConfiguration('topic_name')),
        ]
    )]

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
    
    pcd_arg = DeclareLaunchArgument(
        'pcd',
        default_value='',
        description='Save data in PCD format (set to true to enable)'
    )
    
    ply_arg = DeclareLaunchArgument(
        'ply',
        default_value='',
        description='Save data in PLY format (set to true to enable)'
    )
    
    laz_arg = DeclareLaunchArgument(
        'laz',
        default_value='',
        description='Save data in LAZ format (requires liblaszip, set to true to enable)'
    )
    
    las_arg = DeclareLaunchArgument(
        'las',
        default_value='',
        description='Save data in LAS format (requires liblaszip, set to true to enable)'
    )
    
    stop_recording_arg = DeclareLaunchArgument(
        'stop_recording',
        default_value='false',
        description='Set to true to stop recording and save data'
    )
    
    # Create the LiDAR recorder node using OpaqueFunction to handle dynamic parameter logic
    lidar_recorder_node = OpaqueFunction(function=create_lidar_recorder_node)
    
    return LaunchDescription([
        output_dir_arg,
        topic_name_arg,
        raw_arg,
        filtered_arg,
        filter_lvl_arg,
        save_individual_files_arg,
        pcd_arg,
        ply_arg,
        laz_arg,
        las_arg,
        stop_recording_arg,
        lidar_recorder_node,
    ])