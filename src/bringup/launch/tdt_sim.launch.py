"""OriginSim nav with the T-DT Z-Localization backend and Point-LIO frontend.

Run the Gazebo branch in a separate sourced workspace. Do not start startup.launch.py
or no_smoother_tunnels.launch.py alongside this launch: each publishes map->odom.
"""

import os

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

    def setup(context):
        prior = LaunchConfiguration('prior_map').perform(context)
        if not os.path.isfile(os.path.join(prior, 'keyframes.txt')):
            raise RuntimeError(
                f'T-DT prior map bundle missing at {prior}; run tools/build_tdt_prior.sh first')
        with open(os.path.join(backend, 'config', 'backend.yaml'), encoding='utf-8') as stream:
            params = yaml.safe_load(stream)
        with open(os.path.join(bringup, 'params', 'tdt_backend.yaml'), encoding='utf-8') as stream:
            merge(params, yaml.safe_load(stream))
        params['prior_map']['enabled'] = True
        params['prior_map']['directory'] = prior
        for name, key in (('initial_x', 'x'), ('initial_y', 'y'),
                          ('initial_z', 'z'), ('initial_yaw_deg', 'yaw_deg')):
            params['startup_hint'][key] = float(LaunchConfiguration(name).perform(context))

        map_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('map_server'), 'launch', 'map_server.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'), 'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file'),
                'static_pcd': '', 'region_yaml': '',
            }.items())
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('point_lio_with_grid_map'), 'launch',
                'point_lio.launch.py')),
                launch_arguments={
                    'point_lio_cfg_dir': os.path.join(
                        get_package_share_directory('point_lio_with_grid_map'), 'config',
                        'velody16.yaml'),
                    'use_sim_time': 'true',
                }.items()),
            Node(package='backend', executable='backend_node', name='backend_node',
                 parameters=[params, {'use_sim_time': True}], output='screen'),
            map_launch,
            Node(package='nav_executor', executable='nav_executor_node',
                 condition=IfCondition(LaunchConfiguration('navigation')),
                 parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True},
                             {'navigation_params_file': LaunchConfiguration('params_file')}],
                 output='screen'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('fake_baselink'), 'launch',
                'fake_baselink.launch.py')),
                launch_arguments={'base_link_frame_id': 'chassis'}.items()),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('prior_map', description='Absolute path to a built T-DT map bundle'),
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
