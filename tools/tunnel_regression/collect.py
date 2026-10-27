#!/usr/bin/env python3
"""ROS adapter, independent sensor gates and JSONL recorder for one trial.

The pure evaluators import no ROS modules. This process owns only test relays
and observations; it publishes the original goal once, and never overrides
motion commands until recording has ended and teardown begins.
"""

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import sys
import time

# Allow direct execution by the supervisor without installing another ROS pkg.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tunnel_regression.harness import CONTACT_TOPIC, INPUT_TOPICS, RAW_TOPICS
from tunnel_regression.metrics import chassis_tilt_deg, classify_contact, evaluate_trial
from tunnel_regression.provenance import binary_fingerprints, sha256, verify_install, write_json
from tunnel_regression.safety import (
    HOLD_SIM_SECONDS, HOLD_WALL_SECONDS, RECOVERY_DEADLINE_SECONDS,
    SENSORS, evaluate_safety, magnitude, moving,
)


PARAMETER_NAMES = {
    '/nav_executor': ['path_smoother.enabled', 'odom_timeout', 'control_timeout',
                      'controller_frequency', 'MPC.dt', 'MPC.horizon',
                      'MPC.collision_margin'],
    '/map_server': ['yaml_filename', 'static_pcd', 'robot_radius', 'safety_margin',
                    'grid_margin_factor', 'dynamic_obstacles_enabled'],
    '/map_client': ['robot_radius', 'require_cloud', 'require_radar'],
}


def stamp_seconds(stamp):
    seconds, nanos = float(stamp.sec), float(stamp.nanosec)
    if (not math.isfinite(seconds) or not math.isfinite(nanos)
            or seconds < 0 or not 0 <= nanos < 1e9):
        raise ValueError('Invalid nonnegative ROS timestamp')
    return seconds + nanos * 1e-9


def vector(value):
    result = [float(value.x), float(value.y), float(value.z)]
    if not all(math.isfinite(number) for number in result):
        raise ValueError('Nonfinite vector in observed ROS message')
    return result


def contacts_payload(message, received_wall):
    """Decode every contact point; incomplete pair/normal/depth data is an error."""
    collisions = []
    for contact in message.contacts:
        if not contact.normals or len(contact.normals) != len(contact.depths):
            raise ValueError('Contact has missing/mismatched normal and depth arrays')
        for normal, depth in zip(contact.normals, contact.depths):
            a, b = contact.collision1.name, contact.collision2.name
            n, d = vector(normal), float(depth)
            if not math.isfinite(d):
                raise ValueError('Nonfinite contact depth')
            collisions.append({'a': a, 'b': b, 'normal': n, 'depth_m': d,
                               'kind': classify_contact(a, b, n, d)})
    if not collisions:
        return None  # An empty message cannot manufacture contact health.
    return {'stamp': stamp_seconds(message.header.stamp),
            'received_wall': received_wall, 'checked': True, 'collisions': collisions}


def path_digest(message):
    points = []
    for pose in message.poses:
        p, q = pose.pose.position, pose.pose.orientation
        points.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    payload = json.dumps(points, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest() if points else None


class Monitor:
    def __init__(self, node, case, directory, manifest, args):
        from geometry_msgs.msg import PoseStamped, Twist
        from map_server.msg import MapStatus
        from nav_msgs.msg import Odometry, Path as NavPath
        from rcl_interfaces.msg import Log
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState, PointCloud2
        from std_msgs.msg import String, UInt64
        from tf2_ros import Buffer, TransformListener

        self.node, self.case, self.directory, self.args = node, case, directory, args
        self.start_wall = time.monotonic()
        self.sim = 0.0
        self.last_clock_wall = self.last_clock_progress_wall = 0.0
        self.last_sample_sim = -1.0
        self.goal_time = self.goal_wall = None
        self.expected_goal_id = 1
        self.goal_id = None
        self.task_state = None
        self.failure = None
        self.done = False
        self.recording_closed = False
        self.blocked = set()
        self.truth = self.command = self.odom = self.contacts = None
        self.status = None
        self.status_received = 0.0
        self.source, self.delivered = {}, {}
        self.forwarded = {name: 0 for name in SENSORS}
        self.counts = {'truth': 0, 'command': 0, 'odom': 0, 'contacts': 0, 'clock': 0}
        self.contact_points_checked = 0
        self.last_contact_written_sim = -1.0
        self.original_paths, self.output_paths = set(), set()
        self.records = []
        self.event_stream = (directory / 'events.jsonl').open('x', encoding='utf-8')
        self.last_flush = 0.0
        self.stationary_since = self.moving_since = None
        self.fault_begin = self.fault_end = None
        self.restore_pose = None
        self.auto_recovered = False
        self.last_graph_check = self.last_evaluation = -1.0
        self.params = {}
        self.param_pending = {}
        self.param_started = {}
        self.metadata = {
            'schema_version': 1, 'execution_backend': 'gazebo',
            'runtime_verified': False, 'map_sha256_ok': False,
            'smoother_enabled': None, 'raw_path_output_identical': None,
            'recording_complete': False,
            'environment': {'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
                            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
                            'GZ_PARTITION': os.environ.get('GZ_PARTITION'),
                            'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', 'default'),
                            'python': platform.python_version(), 'platform': platform.platform()},
            'source_manifest_sha256': sha256(args.manifest),
            'installed_files': verify_install(manifest),
            'installed_binaries': binary_fingerprints(),
            'ground_truth_usage': 'Only this observer; navigation receives gated physical sensors.'}
        self.manifest = manifest
        write_json(directory / 'metadata.json', self.metadata)
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_state = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_contact = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        qos_log = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.PoseStamped, self.Twist = PoseStamped, Twist
        self.goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.relays = {}
        for name, message_type in (('lidar', PointCloud2), ('imu', Imu), ('wheels', JointState)):
            # Consumer-side gates leave all Gazebo sources and truth/clock alive.
            qos = QoSProfile(depth=200 if name == 'imu' else 10,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.relays[name] = node.create_publisher(message_type, INPUT_TOPICS[name], qos)
            node.create_subscription(message_type, RAW_TOPICS[name],
                                     lambda msg, name=name: self.on_source(name, msg), qos)
            node.create_subscription(message_type, INPUT_TOPICS[name],
                                     lambda msg, name=name: self.on_input(name, msg), qos)
        node.create_subscription(Clock, '/clock', self.on_clock, qos_sensor)
        node.create_subscription(Odometry, '/Odometry', self.on_truth, qos_sensor)
        node.create_subscription(Odometry, '/odom', self.on_odom, qos_sensor)
        node.create_subscription(Twist, '/cmd_vel_chassis', self.on_command, qos_sensor)
        node.create_subscription(Contacts, CONTACT_TOPIC, self.on_contacts, qos_contact)
        node.create_subscription(Log, '/rosout', self.on_log, qos_log)
        node.create_subscription(UInt64, '/nav_executor/goal_id', self.on_goal_id, qos_state)
        node.create_subscription(String, '/nav_executor/recovery_status',
                                 lambda msg: self.record('recovery', status=msg.data), qos_state)
        node.create_subscription(MapStatus, '/map_server/status', self.on_status, qos_state)
        for topic, key in (('/plan_original', 'original'), ('/plan_smoothed', 'output')):
            node.create_subscription(NavPath, topic,
                                     lambda msg, key=key: self.on_path(key, msg), qos_state)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, node)
        self.parameter_names = copy.deepcopy(PARAMETER_NAMES)
        self.parameter_clients = {name: AsyncParameterClient(node, name)
                                  for name in self.parameter_names}

    def wall(self):
        return time.monotonic() - self.start_wall

    def record(self, kind, **fields):
        if self.recording_closed:
            return
        entry = {'type': kind, 'sim_time': self.sim, 'wall_time': self.wall(), **fields}
        # Detach cached messages: a later callback must not rewrite history.
        entry = copy.deepcopy(entry)
        self.event_stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
        self.records.append(entry)
        if kind != 'sample' or self.wall() - self.last_flush > 1:
            self.event_stream.flush()
            self.last_flush = self.wall()

    def on_clock(self, msg):
        value = stamp_seconds(msg.clock)
        now = self.wall()
        if value < self.sim - 0.05:
            self.failure = 'Simulation clock went backwards'
        if value > self.sim:
            self.last_clock_progress_wall = now
        self.sim, self.last_clock_wall = value, now
        self.counts['clock'] += 1

    def _observe_sensor(self, target, name, msg):
        target[name] = {'stamp': stamp_seconds(msg.header.stamp),
                        'received_sim': self.sim, 'received_wall': self.wall(),
                        'count': target.get(name, {}).get('count', 0) + 1}

    def on_source(self, name, msg):
        self._observe_sensor(self.source, name, msg)
        if name not in self.blocked:
            self.relays[name].publish(msg)
            self.forwarded[name] += 1

    def on_input(self, name, msg):
        self._observe_sensor(self.delivered, name, msg)

    def on_truth(self, msg):
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        position, quaternion = vector(p), [float(q.x), float(q.y), float(q.z), float(q.w)]
        chassis_tilt_deg(quaternion)  # Reject malformed data before it enters cached evidence.
        observed = {'stamp': stamp_seconds(msg.header.stamp), 'received_wall': self.wall(),
                    'x': position[0], 'y': position[1], 'z': position[2],
                    'q': quaternion, 'linear': vector(twist.linear),
                    'angular': vector(twist.angular)}
        self.truth = observed
        self.counts['truth'] += 1

    def on_odom(self, msg):
        # Existing launch fixes map->odom translation with yaw=0.
        p = vector(msg.pose.pose.position)
        self.odom = {'stamp': stamp_seconds(msg.header.stamp), 'received_wall': self.wall(),
                     'received_sim': self.sim,
                     'x': p[0] + self.case['start'][0],
                     'y': p[1] + self.case['start'][1]}
        self.counts['odom'] += 1

    def on_command(self, msg):
        self.command = {'received_sim': self.sim, 'received_wall': self.wall(),
                        'linear': vector(msg.linear), 'angular': vector(msg.angular)}
        self.counts['command'] += 1
        # Preserve every command, so short unsafe pulses cannot hide in samples.
        self.record('command', linear=self.command['linear'], angular=self.command['angular'])

    def on_contacts(self, msg):
        payload = contacts_payload(msg, self.wall())
        if payload is None:
            return
        self.contacts = payload
        self.counts['contacts'] += 1
        self.contact_points_checked += len(payload['collisions'])
        forbidden = any(c['kind'] == 'forbidden' for c in payload['collisions'])
        # Classify every received point; store every violation, rate-limit only
        # redundant support observations. Samples also retain the latest payload.
        if forbidden or self.sim - self.last_contact_written_sim >= 0.05:
            self.record('contact', contacts=payload)
            self.last_contact_written_sim = self.sim
        if forbidden and self.goal_time is not None:
            self.failure = 'Forbidden physical contact after goal'

    def on_goal_id(self, msg):
        self.goal_id = int(msg.data)
        if self.goal_time is not None and self.goal_id not in (0, self.expected_goal_id):
            self.failure = 'Another goal replaced the trial goal'

    def on_log(self, msg):
        if msg.name != 'nav_executor' and not msg.name.startswith('nav_executor.'):
            return
        text = msg.msg
        self.record('nav_log', text=text, level=int(msg.level))
        task = re.search(r'Task state=(\w+) goal_id=(\d+)', text)
        if task:
            state, goal = task.group(1), int(task.group(2))
            if self.goal_time is not None and goal == self.expected_goal_id:
                self.task_state = state
                self.record('task', state=state, goal_id=goal)
        if 'Executing full path:' in text:
            self.record('execution', text=text)
        if 'Path execution stopped:' in text and 'Replacing execution path' not in text:
            self.record('interruption', reason=text.split('Path execution stopped:', 1)[1].strip())

    def on_status(self, msg):
        self.status, self.status_received = msg, self.wall()

    def on_path(self, key, msg):
        digest = path_digest(msg)
        if digest:
            (self.original_paths if key == 'original' else self.output_paths).add(digest)
            self.record('path', channel=key, sha256=digest, pose_count=len(msg.poses))

    def snapshot(self):
        return {'truth': self.truth, 'command': self.command, 'odom': self.odom,
                'contacts': self.contacts, 'sensors': {'source': self.source, 'input': self.delivered},
                'forwarded_counts': self.forwarded}

    def fresh(self, data, sim_age=0.25, wall_age=2, stamp='stamp'):
        if not data:
            return False
        return (-0.05 <= self.sim - data[stamp] <= sim_age and
                0 <= self.wall() - data['received_wall'] <= wall_age)

    def graph_ok(self):
        for name in SENSORS:
            raw = self.node.get_publishers_info_by_topic(RAW_TOPICS[name])
            gated = self.node.get_publishers_info_by_topic(INPUT_TOPICS[name])
            consumers = self.node.get_subscriptions_info_by_topic(INPUT_TOPICS[name])
            if len(raw) != 1 or len(gated) != 1 or gated[0].node_name != self.node.get_name():
                return False
            if not any(item.node_name == 'wheel_lidar_odometry' for item in consumers):
                return False
            if any(item.node_name == 'wheel_lidar_odometry'
                   for item in self.node.get_subscriptions_info_by_topic(RAW_TOPICS[name])):
                return False
        if any(item.node_name != self.node.get_name()
               for item in self.node.get_subscriptions_info_by_topic('/Odometry')):
            return False
        return (self.node.count_publishers('/clock') == 1
                and self.node.count_publishers('/Odometry') == 1
                and self.node.count_publishers('/odom') == 1
                and self.node.count_publishers('/cmd_vel') == 1
                and self.node.count_publishers('/cmd_vel_chassis') == 1
                and self.node.count_publishers('/goal_pose') == 1
                and self.node.count_publishers(CONTACT_TOPIC) == 1
                and self.goal_publisher.get_subscription_count() == 1)

    def update_parameters(self):
        from rclpy.parameter import parameter_value_to_python
        for name, client in self.parameter_clients.items():
            if name in self.params:
                continue
            future = self.param_pending.get(name)
            if future is None:
                if client.services_are_ready():
                    self.param_pending[name] = client.get_parameters(self.parameter_names[name])
                    self.param_started[name] = self.wall()
            elif future.done():
                response = future.result()
                if response is None or len(response.values) != len(self.parameter_names[name]):
                    raise RuntimeError('Missing parameter response from ' + name)
                self.params[name] = dict(zip(self.parameter_names[name],
                                             map(parameter_value_to_python, response.values)))
            elif self.wall() - self.param_started[name] > 10:
                raise TimeoutError('Parameter query timed out: ' + name)
        if len(self.params) != len(self.parameter_names):
            return False
        if self.metadata['map_sha256_ok']:
            return True
        nav, server, client = (self.params[name] for name in ('/nav_executor', '/map_server', '/map_client'))
        expected_nav = {'path_smoother.enabled': False, 'odom_timeout': 0.5,
                        'control_timeout': 0.2, 'controller_frequency': 20.0,
                        'MPC.dt': 0.05, 'MPC.horizon': 10, 'MPC.collision_margin': 0.01}
        expected_server = {'static_pcd': '', 'robot_radius': 0.3, 'safety_margin': 0.01,
                           'grid_margin_factor': 0.4, 'dynamic_obstacles_enabled': False}
        expected_client = {'robot_radius': 0.3, 'require_cloud': False, 'require_radar': False}
        for actual, expected in ((nav, expected_nav), (server, expected_server), (client, expected_client)):
            for name, value in expected.items():
                if actual.get(name) != value:
                    raise RuntimeError(f'Unexpected live parameter {name}: {actual.get(name)!r}')
        import yaml
        map_yaml = Path(server['yaml_filename']).resolve()
        yaml_data = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
        image = (map_yaml.parent / yaml_data['image']).resolve()
        preserved = self.manifest['protected_inputs']
        if sha256(image) != preserved['nav/map_pgm']['sha256']:
            raise RuntimeError('MapServer selected a different PGM')
        if sha256(map_yaml) != preserved['nav/src/bringup/map/rmuc_2026.yaml']['sha256']:
            raise RuntimeError('MapServer selected different map YAML calibration')
        # navigation_params_file exists only on the short-lived bootstrap Node
        # in nav_executor_node.cpp, which is destroyed before the live executor
        # is created. It cannot honestly be queried from the running executor.
        # The protected installed launch selects this file; active critical
        # values above are independently queried from the running nodes.
        from ament_index_python.packages import get_package_share_directory
        params_path = (Path(get_package_share_directory('bringup')) /
                       'params/no_smoother_tunnels.yaml').resolve()
        if sha256(params_path) != preserved['nav/src/bringup/params/no_smoother_tunnels.yaml']['sha256']:
            raise RuntimeError('Executor selected a different navigation configuration')
        self.metadata.update(map_sha256_ok=True, smoother_enabled=False,
                             map_yaml=str(map_yaml), map_pgm_sha256=sha256(image),
                             controller_collision_margin_m=nav['MPC.collision_margin'],
                             navigation_params_file=str(params_path),
                             navigation_params_provenance='Protected installed launch default; '
                             'bootstrap-only filename parameter is not available on the live node',
                             actual_parameters=self.params)
        write_json(self.directory / 'metadata.json', self.metadata)
        return True

    def ready(self):
        from rclpy.time import Time
        previous_stationary_since = self.stationary_since
        # Missing/stale prerequisites break continuity of the initial witness.
        self.stationary_since = None
        if not self.update_parameters():
            return False
        if not all(self.fresh(value) for value in (self.truth, self.odom)):
            return False
        if not self.fresh(self.command, stamp='received_sim') or not self.fresh(self.contacts, 2, 5):
            return False
        if not all(self.fresh(stream.get(name), 0.35, 5)
                   for stream in (self.source, self.delivered) for name in SENSORS):
            return False
        if (self.status is None or self.status.health.map_error or self.status.worker_error
                or self.status.derived_version == 0 or self.wall() - self.status_received > 2):
            return False
        if self.goal_id != 0 or not self.graph_ok():
            return False
        if any(c['kind'] != 'support' for c in self.contacts['collisions']):
            return False
        try:
            transform = self.buffer.lookup_transform('map', 'base_link_fake', Time())
        except Exception:
            return False
        t = transform.transform.translation
        if math.dist((t.x, t.y), self.case['start']) > 0.05:
            raise RuntimeError('Initial localization does not match requested calibrated start')
        if math.dist((self.truth['x'], self.truth['y']), self.case['start']) > 0.03:
            raise RuntimeError('Actual Gazebo spawn differs from requested offset')
        if abs(self.truth['z'] - self.case['initial_z']) > 0.06:
            return False  # Wait for the spawned model to settle.
        stationary = (magnitude(self.truth['linear']) < 0.02 and
                      magnitude(self.truth['angular']) < 0.02 and
                      magnitude(self.command['linear']) < 0.02 and
                      magnitude(self.command['angular']) < 0.02 and
                      chassis_tilt_deg(self.truth['q']) < 5.0)
        if not stationary:
            self.stationary_since = None
            return False
        self.stationary_since = (previous_stationary_since
                                 if previous_stationary_since is not None else self.sim)
        return self.sim - self.stationary_since >= 1.0

    def send_goal(self):
        self.metadata['runtime_verified'] = True
        self.metadata['contact_world_sha256'] = sha256(self.directory / 'contact-world.sdf')
        self.metadata['contacts_bridge_sha256'] = sha256(self.directory / 'contacts-bridge.yaml')
        write_json(self.directory / 'metadata.json', self.metadata)
        self.goal_time, self.goal_wall = self.sim, self.wall()
        self.record('goal', goal_id=self.expected_goal_id, xy=self.case['goal'])
        # Initial witness before any movement distinguishes all three offsets.
        self.record('sample', **self.snapshot())
        self.last_sample_sim = self.sim
        msg = self.PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.case['goal']
        msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(msg)

    def fault_snapshot(self):
        return {'sensor': self.case['fault_sensor'],
                'source_counts': {name: self.source[name]['count'] for name in SENSORS},
                'input_counts': {name: self.delivered[name]['count'] for name in SENSORS},
                'forwarded_counts': dict(self.forwarded)}

    def update_fault(self):
        if self.case['kind'] != 'sensor_dropout':
            return
        snapshot = {'sim_time': self.sim, 'wall_time': self.wall(), **self.snapshot()}
        sensor = self.case['fault_sensor']
        if self.fault_begin is None:
            if (moving(snapshot) and self.fresh(self.truth)
                    and self.fresh(self.command, stamp='received_sim')):
                if self.moving_since is None:
                    self.moving_since = self.sim
                if self.sim - self.moving_since >= 0.3:
                    self.fault_begin = (self.sim, self.wall())
                    self.blocked.add(sensor)
                    self.record('fault_started', **self.fault_snapshot())
            else:
                self.moving_since = None
            if self.sim - self.goal_time > 20:
                raise TimeoutError('No sustained translation for independent fault injection')
        elif self.fault_end is None:
            if (self.sim - self.fault_begin[0] >= HOLD_SIM_SECONDS and
                    self.wall() - self.fault_begin[1] >= HOLD_WALL_SECONDS):
                self.record('fault_ended', **self.fault_snapshot())
                self.fault_end = (self.sim, self.wall())
                self.restore_pose = [self.truth['x'], self.truth['y']]
                self.blocked.remove(sensor)  # Discarded messages are never replayed.
        elif not self.auto_recovered:
            displacement = math.dist(self.restore_pose,
                                     [self.truth['x'], self.truth['y']])
            if (moving(snapshot) and self.fresh(self.truth)
                    and self.fresh(self.command, stamp='received_sim')
                    and self.fresh(self.odom, 0.5, 2)
                    and self.fresh(self.delivered[sensor], 0.35, 5)
                    and self.odom['stamp'] > self.fault_end[0]
                    and self.delivered[sensor]['stamp'] > self.fault_end[0]
                    and displacement >= 0.1):
                self.auto_recovered = True
                self.record('auto_recovered', sensor=sensor, displacement_m=displacement,
                            goal_id=self.expected_goal_id)
            elif (self.sim - self.fault_end[0] > RECOVERY_DEADLINE_SECONDS or
                  self.wall() - self.fault_end[1] > 60):
                raise TimeoutError('Original goal did not automatically resume after restoring input')

    def evaluate(self):
        if self.original_paths or self.output_paths:
            self.metadata['raw_path_output_identical'] = bool(
                self.original_paths and self.output_paths
                and self.original_paths == self.output_paths)
        row = evaluate_trial(self.case, self.records, self.metadata)
        if self.case['kind'] == 'sensor_dropout':
            row = evaluate_safety(self.case, self.records, row)
        return row

    def tick(self):
        if self.failure:
            raise RuntimeError(self.failure)
        if (self.directory / 'abort.json').exists():
            raise RuntimeError(json.loads((self.directory / 'abort.json').read_text())['failure'])
        if self.goal_time is None:
            if self.wall() > self.args.startup_timeout:
                raise TimeoutError('Startup/calibration/fresh sensor/graph/contact readiness timeout')
            if self.ready():
                self.send_goal()
            return
        if self.wall() - self.last_clock_progress_wall > 5:
            raise TimeoutError('Simulation clock stopped progressing')
        if self.sim - self.goal_time > self.args.sim_timeout or self.wall() - self.goal_wall > self.args.trial_timeout:
            raise TimeoutError('Trial exceeded simulation or wall deadline')
        if self.wall() - self.last_graph_check > 1:
            if not self.graph_ok():
                raise RuntimeError('Sensor/command/truth graph changed or an input gate was bypassed')
            self.last_graph_check = self.wall()
        if self.sim - self.last_sample_sim >= 0.049:
            self.record('sample', **self.snapshot())
            self.last_sample_sim = self.sim
            if self.truth is not None and chassis_tilt_deg(self.truth['q']) >= self.case['criteria']['maximum_tilt_deg']:
                raise RuntimeError('Chassis tilt exceeded the regression limit')
            self.update_fault()
        if self.task_state == 'SUCCEEDED' and self.wall() - self.last_evaluation > 0.5:
            self.last_evaluation = self.wall()
            if self.case['kind'] == 'sensor_dropout' and not self.auto_recovered:
                raise RuntimeError('Task completed before independent fault and automatic recovery were proven')
            if self.evaluate()['passed']:
                self.done = True

    def finish(self, error=None):
        if error:
            self.metadata['failure'] = str(error)
            self.record('timeout' if isinstance(error, TimeoutError) else 'error', reason=str(error))
        # Recording ends BEFORE teardown zeros. They cannot satisfy safety checks.
        self.metadata.update(recording_complete=True, counts=self.counts,
                             source_counts={k: v['count'] for k, v in self.source.items()},
                             delivered_counts={k: v['count'] for k, v in self.delivered.items()},
                             forwarded_counts=self.forwarded,
                             contact_points_checked=self.contact_points_checked,
                             original_path_hashes=sorted(self.original_paths),
                             output_path_hashes=sorted(self.output_paths))
        row = self.evaluate()
        if not self.original_paths or not self.output_paths:
            self.metadata['raw_path_output_identical'] = False
            row['passed'] = False
            row.setdefault('failure_reasons', []).append('path_output_evidence_missing')
        row.update(kind=self.case['kind'], execution_backend='gazebo',
                   execution_status='completed' if error is None else 'error',
                   events='events.jsonl', metadata='metadata.json')
        if error:
            row['passed'] = False
        self.event_stream.flush()
        self.event_stream.close()
        self.recording_closed = True
        row['events_sha256'] = sha256(self.directory / 'events.jsonl')
        write_json(self.directory / 'metadata.json', self.metadata)
        write_json(self.directory / 'result.json', row)
        self.blocked.clear()
        return row

    def teardown_stop(self):
        # This publisher exists only after recording has ended. It is never a
        # controller, a fake watchdog, or part of safety-stop evidence.
        publisher = self.node.create_publisher(self.Twist, '/cmd_vel_chassis', 10)
        import rclpy
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and rclpy.ok():
            publisher.publish(self.Twist())
            rclpy.spin_once(self.node, timeout_sec=0.02)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-dir', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--startup-timeout', type=float, default=180)
    parser.add_argument('--trial-timeout', type=float, default=240)
    parser.add_argument('--sim-timeout', type=float, default=90)
    args = parser.parse_args(argv)
    case = json.loads((args.trial_dir / 'case.json').read_text(encoding='utf-8'))
    manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tunnel_regression_monitor', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    monitor = None
    result = {'passed': False}
    stop = {'requested': False}

    def interrupted(_signal, _frame):
        stop['requested'] = True

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Discover existing processes before the runner starts its own stack.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        occupied = [topic for topic in ('/clock', '/Odometry', '/odom', '/cmd_vel_chassis')
                    if node.count_publishers(topic)]
        if occupied:
            raise RuntimeError('Selected ROS domain is occupied: ' + ', '.join(occupied))
        monitor = Monitor(node, case, args.trial_dir, manifest, args)
        write_json(args.trial_dir / 'collector-ready.json', {'ready': True})
        error = None
        try:
            while rclpy.ok() and not monitor.done:
                if stop['requested']:
                    raise RuntimeError('Collector interrupted')
                rclpy.spin_once(node, timeout_sec=0.01)
                monitor.tick()
        except Exception as exc:
            error = exc
        result = monitor.finish(error)
    finally:
        if monitor is not None:
            monitor.blocked.clear()
            if monitor.recording_closed:
                monitor.teardown_stop()
            elif not monitor.event_stream.closed:
                monitor.event_stream.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
