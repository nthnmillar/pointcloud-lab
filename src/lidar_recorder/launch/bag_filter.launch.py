#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    input_bag_arg = DeclareLaunchArgument(
        'input_bag',
        default_value='',
        description='Input bag file path (required)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='filtered_lidar_data',
        description='Output directory for filtered data'
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
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable PCL visualization'
    )
    
    # Spatial filter parameters
    filter_x_min_arg = DeclareLaunchArgument(
        'filter_x_min',
        default_value='-50.0',
        description='Minimum X coordinate for spatial filtering'
    )
    
    filter_x_max_arg = DeclareLaunchArgument(
        'filter_x_max',
        default_value='50.0',
        description='Maximum X coordinate for spatial filtering'
    )
    
    filter_y_min_arg = DeclareLaunchArgument(
        'filter_y_min',
        default_value='-50.0',
        description='Minimum Y coordinate for spatial filtering'
    )
    
    filter_y_max_arg = DeclareLaunchArgument(
        'filter_y_max',
        default_value='50.0',
        description='Maximum Y coordinate for spatial filtering'
    )
    
    filter_z_min_arg = DeclareLaunchArgument(
        'filter_z_min',
        default_value='-10.0',
        description='Minimum Z coordinate for spatial filtering'
    )
    
    filter_z_max_arg = DeclareLaunchArgument(
        'filter_z_max',
        default_value='10.0',
        description='Maximum Z coordinate for spatial filtering'
    )
    
    # Bag filter node
    bag_filter_node = Node(
        package='lidar_recorder',
        executable='bag_filter_node',
        name='bag_filter',
        output='screen',
        parameters=[{
            'input_bag': LaunchConfiguration('input_bag'),
            'output_dir': LaunchConfiguration('output_dir'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'remove_outliers': LaunchConfiguration('remove_outliers'),
            'outlier_std_dev': LaunchConfiguration('outlier_std_dev'),
            'outlier_min_neighbors': LaunchConfiguration('outlier_min_neighbors'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'filter_x_min': LaunchConfiguration('filter_x_min'),
            'filter_x_max': LaunchConfiguration('filter_x_max'),
            'filter_y_min': LaunchConfiguration('filter_y_min'),
            'filter_y_max': LaunchConfiguration('filter_y_max'),
            'filter_z_min': LaunchConfiguration('filter_z_min'),
            'filter_z_max': LaunchConfiguration('filter_z_max'),
        }]
    )
    
    return LaunchDescription([
        input_bag_arg,
        output_dir_arg,
        voxel_size_arg,
        remove_outliers_arg,
        outlier_std_dev_arg,
        outlier_min_neighbors_arg,
        enable_visualization_arg,
        filter_x_min_arg,
        filter_x_max_arg,
        filter_y_min_arg,
        filter_y_max_arg,
        filter_z_min_arg,
        filter_z_max_arg,
        bag_filter_node,
    ])

