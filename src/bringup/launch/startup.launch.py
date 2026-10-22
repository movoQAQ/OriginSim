"""Start localization, maps, visualization and point cloud processing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('bringup')
    params_file = os.path.join(bringup_dir, 'params', 'params.yaml')

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        parameters=[params_file, {'autostart': True}],
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('point_lio_with_grid_map'),
            'launch', 'point_lio.launch.py'))
    )

    start_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(bringup_dir, 'rviz', 'default_view.rviz'),
            '--ros-args', '--log-level', 'error',
        ],
        parameters=[{'use_sim_time': True}],
        output='log',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    modify = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('modify_map_to_odom'), 'launch', 'modify.launch.py'))
    )

    load_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('map_server'), 'launch', 'map_server.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': params_file,
            'static_pcd': LaunchConfiguration('static_pcd'),
            'region_yaml': LaunchConfiguration('region_yaml'),
        }.items(),
    )

    fake_baselink = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fake_baselink'), 'launch', 'fake_baselink.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(bringup_dir, 'map', 'rmuc_2027.yaml')),
        DeclareLaunchArgument('static_pcd', default_value=os.path.join(bringup_dir, 'map', 'rmuc_2027.pcd')),
        DeclareLaunchArgument('region_yaml', default_value=''),
        pointlio,
        container,
        modify,
        start_rviz,
        load_map_server,
        fake_baselink,
    ])
