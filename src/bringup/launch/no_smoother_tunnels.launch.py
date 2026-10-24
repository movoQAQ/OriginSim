"""Original PGM navigation with calibrated wheel/IMU/fixed-wall LiDAR fusion."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('bringup')
    params = LaunchConfiguration('params_file')
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            bringup, 'params', 'no_smoother_tunnels.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bringup, 'map', 'rmuc_2026.yaml')),
        DeclareLaunchArgument('initial_x', default_value='3.4'),
        DeclareLaunchArgument('initial_y', default_value='9.5'),
        DeclareLaunchArgument('initial_z', default_value='0.2758',
                              description='Calibrated chassis height; planar odometry does not estimate heave'),
        Node(package='map_server', executable='map_server_node', name='map_server',
             parameters=[params, sim_time, {'yaml_filename': LaunchConfiguration('map'),
                                           'static_pcd': ''}]),
        Node(package='nav_executor', executable='nav_executor_node',
             parameters=[params, sim_time, {'navigation_params_file': params}]),
        Node(package='landmark_alignment', executable='wheel_lidar_odometry.py',
             parameters=[sim_time], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', LaunchConfiguration('initial_x'),
                        '--y', LaunchConfiguration('initial_y'),
                        '--z', LaunchConfiguration('initial_z'), '--yaw', '0',
                        '--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim_time]),
        Node(package='fake_baselink', executable='fake_baselink_node',
             parameters=[sim_time, {'base_link_frame_id': 'wheel_base',
                                    'ground_heading': True,
                                    'heading_hold_gain': 1.5,
                                    'heading_hold_max_rate': 0.8,
                                    'align_heading_to_motion': True}]),
    ])
