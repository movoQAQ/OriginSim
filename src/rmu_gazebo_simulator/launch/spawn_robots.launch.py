import os
import json
import importlib.util
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.xmacro4sdf import XMLMacro4sdf

from sdformat_tools.urdf_generator import UrdfGenerator


def spawn_robots(context):
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = os.path.join(
        pkg_pb2025_robot_description,
        "resource",
        "xmacro",
        "pb2025_sentry_robot.sdf.xmacro",
    )
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # Get spawn robot init pose
    gz_world_path = LaunchConfiguration("gz_world_path").perform(context)
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        profile_path = LaunchConfiguration("vehicle_physics_profile").perform(context)
        if profile_path:
            spec = importlib.util.spec_from_file_location(
                "vehicle_physics_profile", os.path.join(pkg_simulator, "launch", "vehicle_physics_profile.py"))
            helper = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(helper)
            with open(profile_path) as profile_file:
                profile = json.load(profile_file)
            robot_doc = ET.fromstring(robot_xml)
            helper.apply_profile(robot_doc, profile)
            robot_xml = ET.tostring(robot_doc, encoding="unicode")

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        # replace the <robot_name> in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            # namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            # namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": False,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "gz",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "gz.msgs.StringMsg",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        ld.add_action(spawn_robot)
        ld.add_action(robot_base)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)
        ld.add_action(set_performer_service)

    return list(ld.entities)


def generate_launch_description():
    simulator = get_package_share_directory("rmu_gazebo_simulator")
    return LaunchDescription([
        DeclareLaunchArgument("gz_world_path", default_value=os.path.join(simulator, "config", "gz_world.yaml")),
        DeclareLaunchArgument("vehicle_physics_profile", default_value="",
                              description="Optional JSON for robot inertials, friction, wheelbase and passive drag"),
        OpaqueFunction(function=spawn_robots),
    ])
