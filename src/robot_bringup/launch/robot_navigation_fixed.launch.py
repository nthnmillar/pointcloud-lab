#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    gazebo_dir = FindPackageShare('yahboom_rosmaster_gazebo')
    localization_dir = FindPackageShare('yahboom_rosmaster_localization')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='True')
    world_file = LaunchConfiguration('world_file', default='test_world.sdf')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='True',
        description='Whether to run SLAM')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value='test_world.sdf',
        description='World file name')
    
    # Start Gazebo with robot
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([gazebo_dir, 'launch', 'yahboom_rosmaster.gazebo.launch.py'])]),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Start EKF for filtered odometry
    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([localization_dir, 'launch', 'ekf_gazebo.launch.py'])]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Start Nav2 without SLAM to avoid TF conflicts
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'False',  # Disable SLAM to avoid TF conflicts
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items()
    )
    
    # Start SLAM separately with controlled TF publishing
    start_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([nav2_bringup_dir, 'launch', 'slam_launch.py'])]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items(),
        condition=IfCondition(slam)
    )
    
    # Static transform publisher to ensure correct TF tree
    # This publishes odom as a root frame (no parent)
    static_odom_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    

    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_ekf_cmd)
    ld.add_action(static_odom_cmd)
    ld.add_action(initial_pose_cmd)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_slam_cmd)
    
    return ld 