"""Headless rmuc_2026 with a robot-only physics profile and original field assets."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulator = share('rmu_gazebo_simulator')
    runtime = os.environ.get('XDG_RUNTIME_DIR', f'/tmp/runtime-{os.getuid()}')
    os.makedirs(runtime, mode=0o700, exist_ok=True)
    return LaunchDescription([
        DeclareLaunchArgument('gz_world_path', default_value=os.path.join(
            simulator, 'config', 'gz_world.yaml')),
        DeclareLaunchArgument('vehicle_physics_profile', default_value=os.path.join(
            simulator, 'config', 'tunnel_vehicle_physics.json')),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('LP_NUM_THREADS', '2'),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', runtime),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(simulator, 'resource', 'models')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': os.path.join(simulator, 'resource', 'worlds',
                                                    'rmuc_2026_world.sdf') +
                              ' -s -r --headless-rendering', 'gz_version': '8'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            simulator, 'launch', 'spawn_robots.launch.py')),
            launch_arguments={
                'gz_world_path': LaunchConfiguration('gz_world_path'),
                'vehicle_physics_profile': LaunchConfiguration('vehicle_physics_profile'),
            }.items()),
        Node(package='ign_sim_pointcloud_tool', executable='ign_sim_pointcloud_tool_node',
             parameters=[{'pcd_topic': 'livox/lidar/pointcloud', 'n_scan': 32,
                          'horizon_scan': 720, 'ang_bottom': 7.0, 'ang_res_y': 1.84375}]),
    ])
