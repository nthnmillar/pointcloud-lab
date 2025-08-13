#!/usr/bin/env python3
"""
SLAM launch file for yahboom rosmaster_x3.
This simply calls the working shell script.
Other users can run: ros2 launch yahboom_rosmaster_bringup slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    """Generate launch description for SLAM."""
    
    # Simply call the working shell script
    slam_script = ExecuteProcess(
        cmd=['./src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_navigation.sh', 'slam'],
        output='screen'
    )
    
    return LaunchDescription([slam_script]) 