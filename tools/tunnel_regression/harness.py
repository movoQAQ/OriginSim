"""Small, ROS-independent launch helpers for the tunnel regression runner.

The installed production launches own all navigation and vehicle settings. This
module only creates a per-trial spawn file and their test wrapper's argv.
"""

import json
import math
from pathlib import Path
import re


ROBOT_NAME = 'red_standard_robot1'
CONTACT_TOPIC = '/tunnel_regression/contacts'
RAW_TOPICS = {
    'lidar': '/livox/lidar/pointcloud',
    'imu': '/livox/imu',
    'wheels': '/joint_states',
}
INPUT_TOPICS = {
    name: '/tunnel_regression/input/' + name for name in RAW_TOPICS
}


def write_spawn_config(path, x, y, spawn_z, robot_name=ROBOT_NAME):
    """Write a standalone YAML-compatible JSON spawn file, yaw fixed at zero.

    ``spawn_z`` is the Gazebo model insertion height, NOT calibrated chassis
    ``initial_z``. The existing red-bottom fixture uses 1.09 vs 0.4758 metres.
    Callers must supply both explicitly and wait for stationary calibration.
    """
    coordinates = (float(x), float(y), float(spawn_z))
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError('Spawn coordinates must be finite')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', robot_name):
        raise ValueError('Invalid Gazebo robot name')
    destination = Path(path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    data = {
        'world': 'rmuc_2026',
        'robots': {'rmuc_2026': [{
            'name': robot_name,
            'color': 'red',
            'x_pose': str(coordinates[0]),
            'y_pose': str(coordinates[1]),
            'z_pose': str(coordinates[2]),
            'yaw': '0.0',
        }]},
    }
    # Exclusive create protects earlier trial inputs from accidental overwrite.
    with destination.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return destination


def launch_command(spawn_config, initial_xyz):
    """Return an argv list; no shell, interpolation, or duplicated tuning."""
    xyz = tuple(float(value) for value in initial_xyz)
    if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
        raise ValueError('initial_xyz must contain three finite coordinates')
    config = Path(spawn_config).resolve()
    if not config.is_file():
        raise FileNotFoundError(config)
    return [
        'ros2', 'launch', str(Path(__file__).with_name('regression.launch.py').resolve()),
        'spawn_config:=' + str(config),
        'initial_x:=' + str(xyz[0]),
        'initial_y:=' + str(xyz[1]),
        'initial_z:=' + str(xyz[2]),
    ]
