from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # remap RTAB-Map topics to your robot’s camera & LiDAR
    remappings = [
        # 3D LiDAR
        ('scan_cloud',      '/lidar/points'),
        ('scan',            '/lidar/points'),
        # RGB camera
        ('rgb/image',       '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        # Depth camera
        ('depth/image',     '/camera/depth/image_raw'),
    ]

    # sensor subscriptions
    sensor_params = {
        'subscribe_rgbd':       False,  # disable combined RGBD
        'subscribe_rgb':        True,
        'subscribe_depth':      True,
        'subscribe_scan':       False,
        'subscribe_scan_cloud': True,
    }

    # SLAM-specific params
    slam_only = {
        'frame_id':              'base_footprint_link',
        'map_frame_id':          'map',
        'odom_sensor_sync':      False,
        'use_action_for_goal':   False,
        'Reg/Force3DoF':         True,
        'Reg/Strategy':          '1',
        'Mem/NotLinkedNodesKept':'false',
    }

    # shared timing/sync
    common = {
        'use_sim_time':    True,
        'approx_sync':     True,
        'sync_queue_size': 20,
        'topic_queue_size':20,
    }

    # build parameter dicts
    slam_params = {**sensor_params, **slam_only}
    viz_params  = {**sensor_params, **common}

    slam_node = Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', output='screen',
        parameters=[slam_params, common],
        remappings=remappings,
        arguments=['-d'],
    )

    viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz',
        name='rtabmap_viz', output='screen',
        parameters=[viz_params],
        remappings=remappings,
    )

    return LaunchDescription([slam_node, viz_node])
