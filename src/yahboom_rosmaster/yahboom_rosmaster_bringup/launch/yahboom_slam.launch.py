#!/usr/bin/env python3
"""
SLAM launch file for yahboom rosmaster_x3.
This simply calls the working shell script.
Other users can run: ros2 launch yahboom_rosmaster_bringup slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for SLAM."""
    
    # Declare the gz_headless argument
    declare_gz_headless_cmd = DeclareLaunchArgument(
        name='gz_headless',
        default_value='false',
        description='Whether to run Gazebo in headless mode - false = show Gazebo GUI (default), true = no GUI but full simulation'
    )
    
    # Get the gz_headless value
    gz_headless = LaunchConfiguration('gz_headless')
    
    # Call the shell script with gz_headless parameter
    slam_script = ExecuteProcess(
        cmd=['./src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_navigation.sh', 'slam', gz_headless],
        output='screen'
    )
    
    return LaunchDescription([
        declare_gz_headless_cmd,
        slam_script
    ]) 