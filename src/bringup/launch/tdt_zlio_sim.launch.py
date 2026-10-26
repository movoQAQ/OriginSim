"""Gazebo localization with Z-Localization's own ZLIO frontend and T-DT backend."""

import atexit
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def generate_launch_description():
    bringup = get_package_share_directory('bringup')
    backend = get_package_share_directory('backend')
    frontend = get_package_share_directory('frontend')

    def setup(context):
        prior = LaunchConfiguration('prior_map').perform(context)
        if not os.path.isfile(os.path.join(prior, 'keyframes.txt')):
            raise RuntimeError(f'T-DT prior map bundle missing at {prior}')

        with open(os.path.join(backend, 'config', 'backend.yaml'), encoding='utf-8') as stream:
            backend_params = yaml.safe_load(stream)
        with open(os.path.join(bringup, 'params', 'tdt_zlio_backend.yaml'), encoding='utf-8') as stream:
            merge(backend_params, yaml.safe_load(stream))
        backend_params['prior_map'].update(enabled=True, directory=prior)
        for name, key in (('initial_x', 'x'), ('initial_y', 'y'),
                          ('initial_z', 'z'), ('initial_yaw_deg', 'yaw_deg')):
            backend_params['startup_hint'][key] = float(LaunchConfiguration(name).perform(context))

        # The SDF places the GPU LiDAR 3 cm above the IMU, both on livox_frame.
        with open(LaunchConfiguration('zlio_config').perform(context), encoding='utf-8') as stream:
            lio_config = yaml.safe_load(stream)
        lio_config['sensor']['body_from_lidar'] = {
            'rotation_xyzw': [0.0, 0.0, 0.0, 1.0],
            'translation': [0.0, 0.0, 0.03],
        }
        # Limit the expensive static-scene startup solve while retaining full 3-D coverage.
        lio_config['initialization']['window_frames'] = 8
        lio_config['initialization']['sample_size'] = 0.2
        lio_config['initialization']['fallback_sample_size'] = 0.1
        lio_config['initialization']['minimum_sampled_points'] = 500
        lio_config['initialization']['maximum_refinements'] = 6
        # Gazebo's nearly noise-free gyro must dominate the LiDAR update during yaw.
        # The hardware default (0.25 rad^2/s^2) made odometry miss a 0.5 rad turn.
        lio_config['filter']['gyro_variance'] = 0.001
        with tempfile.NamedTemporaryFile(mode='w', prefix='tdt-zlio-', suffix='.yaml',
                                         delete=False, encoding='utf-8') as stream:
            yaml.safe_dump(lio_config, stream)
            lio_config_path = stream.name
        atexit.register(lambda: os.path.exists(lio_config_path) and os.unlink(lio_config_path))

        map_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('map_server'), 'launch', 'map_server.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'), 'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file'),
                'static_pcd': '', 'region_yaml': '',
            }.items())
        return [
            Node(package='bringup', executable='zlio_cloud_adapter.py', output='screen',
                 parameters=[{'use_sim_time': True, 'stride': 12}]),
            Node(package='frontend', executable='zlio_node', name='zlio',
                 parameters=[{
                     'config_file': lio_config_path,
                     'input_type': 'pointcloud2',
                     'lidar_topic': '/zlio/points',
                     'point_time_field': 'time',
                     'point_time_scale': 1.0,
                     'imu_topic': '/livox/imu',
                     'imu_acceleration_scale': 1.0,
                     'maximum_imu_interval': 0.2,
                     'maximum_queue_events': 32768,
                     'maximum_buffered_points': 2000000,
                     'world_frame': 'odom', 'body_frame': 'imu',
                     'coordinated_reset': True, 'use_sim_time': True,
                 }], remappings=[('odometry', '/odom')], output='screen'),
            Node(package='backend', executable='backend_node', name='backend_node',
                 parameters=[backend_params, {'use_sim_time': True}], output='screen'),
            map_launch,
            Node(package='nav_executor', executable='nav_executor_node',
                 condition=IfCondition(LaunchConfiguration('navigation')),
                 parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True},
                             {'navigation_params_file': LaunchConfiguration('params_file')}],
                 output='screen'),
            # Use a chassis estimate derived from ZLIO, never Gazebo's truth odom->chassis TF.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '0', '--y', '0', '--z', '-0.6',
                            '--yaw', '1.095', '--frame-id', 'imu',
                            '--child-frame-id', 'chassis_est']),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('fake_baselink'), 'launch',
                'fake_baselink.launch.py')),
                launch_arguments={'base_link_frame_id': 'chassis_est'}.items()),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('prior_map', description='Absolute path to a built T-DT map bundle'),
        DeclareLaunchArgument('zlio_config', default_value=os.path.join(
            frontend, 'config', 'lio.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bringup, 'map', 'rmuc_2026.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            bringup, 'params', 'params.yaml')),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='3.4'),
        DeclareLaunchArgument('initial_y', default_value='9.5'),
        DeclareLaunchArgument('initial_z', default_value='0.88'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='-62.4'),
        OpaqueFunction(function=setup),
    ])
