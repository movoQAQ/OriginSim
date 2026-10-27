"""Test wrapper around the existing production simulator and navigation launches.

Run through tools/run_tunnel_regression.py. The collector provides the independent
three-channel relay and gates /tunnel_regression/input/{lidar,imu,wheels}.
Gazebo's original /Odometry and /clock remain outside those gates.
"""

import importlib.util
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _simulation(context):
    """Preload Contact before dynamically spawning its observed robot sensors."""
    simulator = get_package_share_directory('rmu_gazebo_simulator')
    spawn_config = Path(LaunchConfiguration('spawn_config').perform(context)).resolve()
    helper_path = Path(simulator) / 'launch/regression_contacts.py'
    spec = importlib.util.spec_from_file_location('regression_contacts', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    world_copy = helper.write_contact_world(
        Path(simulator) / 'resource/worlds/rmuc_2026_world.sdf',
        spawn_config.with_name('contact-world.sdf'))
    bridge_config = spawn_config.with_name('contacts-bridge.yaml')
    # Jazzy ros_gz_bridge config-file queues use reliable KeepLast by default.
    # Retain short contact pulses while the collector drains and classifies them.
    with bridge_config.open('x', encoding='utf-8') as stream:
        json.dump([{
            'ros_topic_name': '/tunnel_regression/contacts',
            'gz_topic_name': '/tunnel_regression/contacts',
            'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
            'gz_type_name': 'gz.msgs.Contacts',
            'direction': 'GZ_TO_ROS',
            'publisher_queue': 1000,
            'subscriber_queue': 1000,
        }], stream, indent=2)
        stream.write('\n')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            simulator, 'launch', 'tunnel_sim.launch.py')),
        launch_arguments={
            'gz_world_path': str(spawn_config),
            'world_sdf_path': str(world_copy),
            'regression_contacts': 'true',
        }.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='tunnel_regression_contact_bridge',
             parameters=[{'use_sim_time': True, 'config_file': str(bridge_config)}],
             output='screen')]


def generate_launch_description():
    bringup = get_package_share_directory('bringup')
    return LaunchDescription([
        DeclareLaunchArgument('spawn_config', description='Per-trial spawn YAML/JSON'),
        DeclareLaunchArgument('initial_x'),
        DeclareLaunchArgument('initial_y'),
        DeclareLaunchArgument('initial_z'),
        OpaqueFunction(function=_simulation),
        GroupAction([
            SetRemap(src='/livox/lidar/pointcloud',
                     dst='/tunnel_regression/input/lidar'),
            SetRemap(src='/livox/imu', dst='/tunnel_regression/input/imu'),
            SetRemap(src='/joint_states', dst='/tunnel_regression/input/wheels'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    bringup, 'launch', 'no_smoother_tunnels.launch.py')),
                launch_arguments={
                    'initial_x': LaunchConfiguration('initial_x'),
                    'initial_y': LaunchConfiguration('initial_y'),
                    'initial_z': LaunchConfiguration('initial_z'),
                }.items()),
        ]),
    ])
