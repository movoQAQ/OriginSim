from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_pcd = os.path.join(get_package_share_directory('bringup'), 'map', 'rmuc_2026.pcd')
    return LaunchDescription([
        DeclareLaunchArgument('pcd_file', default_value=default_pcd),
        LoadComposableNodes(
            target_container='container',
            composable_node_descriptions=[ComposableNode(
                package='small_gicp_localization',
                plugin='small_gicp_localization::SmallGicpLocalization',
                name='small_gicp_localization_node',
                parameters=[{
                    'use_sim_time': True,
                    'num_threads': 4,
                    'num_neighbors': 20,
                    'global_leaf_size': 0.25,
                    'registered_leaf_size': 0.25,
                    'max_dist_sq': 1.0,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_link_frame': 'aft_mapped',
                    'pcd_file': LaunchConfiguration('pcd_file'),
                    'cloud_registered_topic': '/cloud_registered',
                    'initial_map_from_odom': [3.4, 9.5, 0.876, -1.095],
                }],
            )],
        ),
    ])
