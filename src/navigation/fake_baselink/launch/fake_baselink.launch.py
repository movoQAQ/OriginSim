
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    fake_vel_transform_node = Node(
        package='fake_baselink',
        executable='fake_baselink_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
            # 'odom_frame_id':'odom',
            # 'forward_distance':0.2
        }]
    )

    return LaunchDescription([DeclareLaunchArgument('base_link_frame_id', default_value='base_link'), fake_vel_transform_node])
