from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time  = LaunchConfiguration('use_sim_time',  default='true')
    localization  = LaunchConfiguration('localization',  default='false')
    rtabmap_viz   = LaunchConfiguration('rtabmap_viz',   default='true')

    rtabmap_parameters = {
        'subscribe_rgbd':   False,      # disable RGB-D
        'subscribe_scan':   False,      # disable 2D laser
        'subscribe_scan_cloud': True,   # enable your 3D pointcloud
        'use_action_for_goal': False,
        'odom_sensor_sync': False,
        'Reg/Force3DoF':      'true',
        'Reg/Strategy':       '1',
        'Mem/NotLinkedNodesKept': 'false',
    }

    # Their shared parameters dict:
    shared_parameters = {
        'frame_id':        'base_footprint_link',
        'use_sim_time':    use_sim_time,
    }

    remappings = [
        ('scan_cloud', '/lidar/points'),
        ('scan',       '/lidar/points'),
    ]

    # SLAM node exactly like them:
    slam_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
        arguments=['-d']
    )

    # Viz node exactly like them:
    viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz', executable='rtabmap_viz',
        name='rtabmap_viz', output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='true',  choices=['true','false']),
        DeclareLaunchArgument('localization',  default_value='false', choices=['true','false']),
        DeclareLaunchArgument('rtabmap_viz',   default_value='true',  choices=['true','false']),
        slam_node,
        viz_node,
    ])
