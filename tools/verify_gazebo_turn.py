#!/usr/bin/env python3
"""Run the MPC/reference against the real swerve Gazebo plugin on flat ground.

The separate ROS domain and Gazebo partition keep this diagnostic independent
of manual RViz goals and RMUC map/STL alignment. It does not test the global
planner, localization, terrain, or dynamic map adapter.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from sdformat_tools.xmacro4sdf import XMLMacro4sdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe-binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--mode', choices=['acute', 'smooth'], default='acute')
    parser.add_argument('--cross-weight', type=float, help='Override the diagnostic solver weight')
    args = parser.parse_args()
    folder = args.output_dir.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(get_package_share_directory('pb2025_robot_description')) /
                           'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
    macro.generate({'global_initial_color': 'red'})
    model = ET.fromstring(macro.to_string()).find('model')
    model.set('name', 'red_standard_robot1')
    model.find('pose').text = '0 0 0.88 0 0 0'
    world = ET.fromstring('''<sdf version="1.9"><world name="default">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<model name="ground"><static>true</static><link name="ground"><collision name="ground">
<geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
</collision></link></model></world></sdf>''')
    world.find('world').append(model)
    ET.ElementTree(world).write(folder / 'world.sdf')
    env = dict(os.environ, ROS_DOMAIN_ID='94', GZ_PARTITION='mpc_turn_' + str(os.getpid()))
    processes, logs = [], []

    def start(name, command):
        log = (folder / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    try:
        start('gz', ['gz', 'sim', '-s', '-r', str(folder / 'world.sdf')])
        start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                        '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'])
        command = [str(args.probe_binary.resolve()), str(folder / 'result.json'), args.mode]
        if args.cross_weight is not None:
            command.append(str(args.cross_weight))
        probe = start('probe', command)
        return probe.wait(timeout=70)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
