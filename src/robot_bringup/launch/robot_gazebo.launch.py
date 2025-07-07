from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = PathJoinSubstitution([robot_description_pkg, 'urdf', 'robot.urdf.xacro'])
    gazebo_config_path = PathJoinSubstitution([robot_bringup_pkg, 'config', 'gazebo_bridge.yaml'])
    rviz_config_path = PathJoinSubstitution([robot_description_pkg, 'rviz', 'urdf_config.rviz'])
    world_path = PathJoinSubstitution([robot_bringup_pkg, 'worlds', 'test_world.sdf'])

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command(['xacro ', urdf_path])}
            ],
            output='screen'
        ),

        # gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': [world_path, ' -r']}.items()
        ),

        # spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description']
        ),

        # ros_gz_bridge with your gazebo_bridge.yaml
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gazebo_config_path}],
            output='screen'
        ),

        # static transform publisher exactly like original XML
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint_link', 'robot/base_footprint_link/lidar']
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
