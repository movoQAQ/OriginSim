"""ROS-free adapter checks using small message/node doubles, not a ROS run."""

import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tunnel_regression.collect import (  # noqa: E402
    Monitor, PARAMETER_NAMES, contacts_payload, path_digest, stamp_seconds, vector,
)
from tunnel_regression.harness import CONTACT_TOPIC, INPUT_TOPICS, RAW_TOPICS  # noqa: E402
from tunnel_regression.metrics import expand_cases  # noqa: E402
from tunnel_regression.provenance import sha256  # noqa: E402
from tunnel_regression.safety import SENSORS  # noqa: E402
from test_metrics import METADATA, passing_records, sample, support_contact  # noqa: E402


def stamp(t):
    seconds = math.floor(t)
    return NS(sec=seconds, nanosec=round((t-seconds)*1e9))


def xyz(values=(0, 0, 0)):
    return NS(x=values[0], y=values[1], z=values[2])


def pose_stamped():
    return NS(header=NS(frame_id='', stamp=stamp(0)),
              pose=NS(position=xyz(), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))


def twist(linear=(0, 0, 0), angular=(0, 0, 0)):
    return NS(linear=xyz(linear), angular=xyz(angular))


def odometry(t=1, position=(0, 0, 0), linear=(0, 0, 0), angular=(0, 0, 0)):
    pose = pose_stamped().pose
    pose.position = xyz(position)
    return NS(header=NS(stamp=stamp(t)), pose=NS(pose=pose),
              twist=NS(twist=twist(linear, angular)))


def contact_message(t=1, pairs=None):
    pairs = [support_contact()] if pairs is None else pairs
    contacts = [NS(collision1=NS(name=p['a']), collision2=NS(name=p['b']),
                   normals=[xyz(p['normal'])], depths=[p['depth_m']]) for p in pairs]
    return NS(header=NS(stamp=stamp(t)), contacts=contacts)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))

    def get_subscription_count(self):
        return 1


class GraphNode:
    def __init__(self, monitor):
        self.monitor = monitor
        self.publishers = {topic: [NS(node_name='gazebo_bridge')] for topic in RAW_TOPICS.values()}
        self.publishers.update({topic: [NS(node_name=self.get_name())] for topic in INPUT_TOPICS.values()})
        self.subscribers = {topic: [NS(node_name='wheel_lidar_odometry'), NS(node_name=self.get_name())]
                            for topic in INPUT_TOPICS.values()}
        self.subscribers.update({topic: [NS(node_name=self.get_name())] for topic in RAW_TOPICS.values()})
        self.subscribers['/Odometry'] = [NS(node_name=self.get_name())]
        self.counts = {topic: 1 for topic in ('/clock', '/Odometry', '/odom', '/cmd_vel',
                                             '/cmd_vel_chassis', '/goal_pose', CONTACT_TOPIC)}

    def get_name(self):
        return 'tunnel_regression_monitor'

    def get_publishers_info_by_topic(self, topic):
        return self.publishers.get(topic, [])

    def get_subscriptions_info_by_topic(self, topic):
        return self.subscribers.get(topic, [])

    def count_publishers(self, topic):
        return self.counts.get(topic, 0)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: stamp(self.monitor.sim)))


class CollectorAdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        m = self.monitor = Monitor.__new__(Monitor)
        m.directory = self.directory
        m.case = {**expand_cases()[0], 'kind': 'traversal'}
        m.sim = m.now_wall = 1.0
        m.wall = lambda: m.now_wall
        m.last_clock_wall = m.last_clock_progress_wall = 0.0
        m.last_sample_sim = -1.0
        m.goal_time = m.goal_wall = None
        m.expected_goal_id = 1
        m.goal_id = 0
        m.task_state = None
        m.failure = None
        m.done = m.recording_closed = False
        m.blocked = set()
        m.records = []
        m.event_stream = (self.directory / 'events.jsonl').open('x', encoding='utf-8')
        m.last_flush = 0.0
        m.last_contact_written_sim = -1.0
        m.counts = {name: 0 for name in ('truth', 'command', 'odom', 'contacts', 'clock')}
        m.contact_points_checked = 0
        m.source = {}
        m.delivered = {}
        m.forwarded = {name: 0 for name in SENSORS}
        m.relays = {name: Publisher() for name in SENSORS}
        m.node = GraphNode(m)
        m.goal_publisher = Publisher()
        m.PoseStamped = pose_stamped
        m.Twist = twist
        m.stationary_since = m.moving_since = None
        m.fault_begin = m.fault_end = None
        m.restore_pose = None
        m.auto_recovered = False
        m.original_paths = m.output_paths = set()
        m.metadata = dict(METADATA)
        m.parameter_names = copy.deepcopy(PARAMETER_NAMES)
        m.params = {}
        m.parameter_clients = {}
        m.param_pending = {}
        m.param_started = {}
        m.status = NS(health=NS(map_error=False), worker_error='', derived_version=1)
        m.status_received = 1.0
        m.buffer = NS(lookup_transform=lambda *a: NS(transform=NS(translation=xyz((*m.case['start'], 0)))))
        m.args = NS(startup_timeout=180, trial_timeout=240, sim_timeout=90)
        m.last_graph_check = m.last_evaluation = -1.0
        self.refresh(1.0)
        (self.directory / 'contact-world.sdf').write_text('<test-only-placeholder/>')
        (self.directory / 'contacts-bridge.yaml').write_text('[]')

    def tearDown(self):
        if not self.monitor.event_stream.closed:
            self.monitor.event_stream.close()
        self.temp.cleanup()

    def refresh(self, t, speed=0.0):
        m = self.monitor
        m.sim = m.now_wall = t
        snapshot = sample(t, *m.case['start'], velocity=[speed, 0, 0], z=m.case['initial_z'])
        for name in ('truth', 'command', 'odom', 'contacts'):
            setattr(m, name, snapshot[name])
        for streams in (m.source, m.delivered):
            for name in SENSORS:
                streams[name] = {'stamp': t, 'received_sim': t, 'received_wall': t, 'count': 1}
        m.status_received = t

    def test_ros_stamp_and_vector_reject_nonfinite_before_cache(self):
        self.assertAlmostEqual(stamp_seconds(NS(sec=2, nanosec=500000000)), 2.5)
        for value in (NS(sec=-1, nanosec=0), NS(sec=0, nanosec=1e9), NS(sec=math.nan, nanosec=0)):
            with self.assertRaises(ValueError):
                stamp_seconds(value)
        with self.assertRaises(ValueError):
            vector(xyz([math.inf, 0, 0]))

    def test_contact_decoder_keeps_all_points_and_reclassifies_body(self):
        body = support_contact()
        body['a'] = 'red_standard_robot1::chassis::base_link_base_collision'
        message = contact_message(pairs=[support_contact(), body])
        message.contacts[0].normals.append(xyz([0, 0, 1]))
        message.contacts[0].depths.append(0.0002)
        decoded = contacts_payload(message, 2.0)
        self.assertEqual(len(decoded['collisions']), 3)
        self.assertEqual([c['kind'] for c in decoded['collisions']], ['support', 'support', 'forbidden'])
        self.assertEqual(decoded['received_wall'], 2.0)
        self.assertIsNone(contacts_payload(contact_message(pairs=[]), 2.0))

    def test_malformed_contact_arrays_and_nonfinite_depth_rejected(self):
        message = contact_message()
        message.contacts[0].depths = []
        with self.assertRaises(ValueError):
            contacts_payload(message, 1.0)
        message = contact_message()
        message.contacts[0].depths[0] = math.nan
        previous = copy.deepcopy(self.monitor.contacts)
        with self.assertRaises(ValueError):
            self.monitor.on_contacts(message)
        self.assertEqual(self.monitor.contacts, previous)

    def test_invalid_truth_and_command_do_not_poison_cached_state(self):
        m = self.monitor
        previous_truth, previous_command = copy.deepcopy(m.truth), copy.deepcopy(m.command)
        message = odometry(position=(math.nan, 0, 0))
        with self.assertRaises(ValueError):
            m.on_truth(message)
        with self.assertRaises(ValueError):
            m.on_command(twist(angular=(0, math.inf, 0)))
        self.assertEqual(m.truth, previous_truth)
        self.assertEqual(m.command, previous_command)
        self.assertEqual(m.records, [])

    def test_truth_adapter_preserves_3d_velocity_and_localization_translation(self):
        m = self.monitor
        message = odometry(1, (1, 2, 3), (4, 5, 6), (0.1, 0.2, 0.3))
        m.on_truth(message)
        self.assertEqual(m.truth['linear'], [4.0, 5.0, 6.0])
        self.assertEqual(m.truth['angular'], [0.1, 0.2, 0.3])
        m.on_odom(message)
        self.assertEqual(m.odom['x'], m.case['start'][0]+1)
        self.assertEqual(m.odom['y'], m.case['start'][1]+2)

    def test_record_detaches_nested_sensor_and_contact_history(self):
        m = self.monitor
        m.record('sample', **m.snapshot())
        original = copy.deepcopy(m.records[0])
        m.truth['x'] += 10
        m.contacts['collisions'][0]['normal'][0] = 20
        m.source['imu']['count'] += 10
        self.assertEqual(m.records[0], original)

    def test_closed_recording_ignores_teardown_command_callbacks(self):
        m = self.monitor
        m.on_command(twist((0.3, 0, 0)))
        m.event_stream.flush()
        before = (self.directory / 'events.jsonl').read_bytes()
        m.recording_closed = True
        m.on_command(twist())
        m.event_stream.flush()
        self.assertEqual((self.directory / 'events.jsonl').read_bytes(), before)
        self.assertEqual(len(m.records), 1)

    def test_every_command_callback_including_short_pulse_is_recorded(self):
        m = self.monitor
        for t, speed in [(1.0, 0), (1.001, 0.3), (1.002, 0)]:
            m.sim = m.now_wall = t
            m.on_command(twist((speed, 0, 0)))
        self.assertEqual([r['linear'][0] for r in m.records], [0.0, 0.3, 0.0])
        self.assertEqual([r['sim_time'] for r in m.records], [1.0, 1.001, 1.002])

    def test_gate_blocks_only_selected_stream_without_replaying(self):
        m = self.monitor
        m.blocked.add('imu')
        message = NS(header=NS(stamp=stamp(1.0)))
        for name in SENSORS:
            m.on_source(name, message)
        self.assertEqual(m.forwarded, {'lidar': 1, 'imu': 0, 'wheels': 1})
        self.assertEqual(m.source['imu']['count'], 2)
        m.blocked.remove('imu')
        self.assertEqual(m.relays['imu'].messages, [])
        m.on_source('imu', NS(header=NS(stamp=stamp(1.1))))
        self.assertEqual(len(m.relays['imu'].messages), 1)
        self.assertAlmostEqual(stamp_seconds(m.relays['imu'].messages[0].header.stamp), 1.1)

    def test_bad_raw_sensor_stamp_does_not_count_or_forward(self):
        m = self.monitor
        before = copy.deepcopy(m.source)
        with self.assertRaises(ValueError):
            m.on_source('imu', NS(header=NS(stamp=NS(sec=math.nan, nanosec=0))))
        self.assertEqual(m.source, before)
        self.assertEqual(m.forwarded['imu'], 0)

    def test_direct_and_duplicate_gate_publishers_rejected(self):
        m = self.monitor
        self.assertTrue(m.graph_ok())
        m.node.subscribers[RAW_TOPICS['imu']].append(NS(node_name='wheel_lidar_odometry'))
        self.assertFalse(m.graph_ok())
        m.node.subscribers[RAW_TOPICS['imu']].pop()
        m.node.publishers[INPUT_TOPICS['imu']].append(NS(node_name='another_relay'))
        self.assertFalse(m.graph_ok())

    def test_duplicate_clock_command_goal_and_truth_consumers_rejected(self):
        m = self.monitor
        for topic in ('/clock', '/cmd_vel', '/cmd_vel_chassis', '/goal_pose'):
            m.node.counts[topic] = 2
            self.assertFalse(m.graph_ok(), topic)
            m.node.counts[topic] = 1
        m.node.subscribers['/Odometry'].append(NS(node_name='wheel_lidar_odometry'))
        self.assertFalse(m.graph_ok())

    def test_log_task_is_filtered_by_source_and_active_goal(self):
        m = self.monitor
        def log(text, name='nav_executor'):
            m.on_log(NS(name=name, msg=text, level=20))
        log('Task state=SUCCEEDED goal_id=1')
        self.assertIsNone(m.task_state)
        m.goal_time = 1.0
        log('Task state=SUCCEEDED goal_id=2')
        log('Task state=SUCCEEDED goal_id=1', 'unrelated_node')
        self.assertIsNone(m.task_state)
        log('Task state=SUCCEEDED goal_id=1 plan_generation=4')
        self.assertEqual(m.task_state, 'SUCCEEDED')
        self.assertEqual(len([r for r in m.records if r['type'] == 'task']), 1)
        m.on_goal_id(NS(data=2))
        self.assertIsNotNone(m.failure)

    def test_replacing_path_is_not_a_spurious_execution_interruption(self):
        m = self.monitor
        for text in ('Path execution stopped: Replacing execution path',
                     'Path execution stopped: Odometry missing or receive timeout',
                     'Executing full path: goal_id=1 generation=2 poses=100'):
            m.on_log(NS(name='nav_executor', msg=text, level=20))
        self.assertEqual(len([r for r in m.records if r['type'] == 'interruption']), 1)
        self.assertEqual(len([r for r in m.records if r['type'] == 'execution']), 1)

    def test_transient_forbidden_contact_not_lost_to_support_rate_limit(self):
        m = self.monitor
        m.goal_time = 1.0
        m.on_contacts(contact_message())
        body = support_contact()
        body['a'] = 'red_standard_robot1::chassis::base_link_base_collision'
        m.sim = m.now_wall = 1.001
        m.on_contacts(contact_message(1.001, [body]))
        m.sim = m.now_wall = 1.002
        m.on_contacts(contact_message(1.002))
        events = [r for r in m.records if r['type'] == 'contact']
        self.assertEqual(len(events), 2)
        self.assertEqual(events[1]['contacts']['collisions'][0]['kind'], 'forbidden')
        self.assertIsNotNone(m.failure)

    def test_clock_and_receive_freshness_are_independent(self):
        m = self.monitor
        data = {'stamp': 1.0, 'received_wall': 1.0}
        self.assertTrue(m.fresh(data))
        m.now_wall = 4.0
        self.assertFalse(m.fresh(data))
        m.now_wall = 1.0
        m.sim = 1.3
        self.assertFalse(m.fresh(data))
        m.sim = 2.0
        m.on_clock(NS(clock=stamp(1.0)))
        self.assertIsNotNone(m.failure)

    def test_send_goal_records_exact_initial_pose_before_publishing_once(self):
        m = self.monitor
        m.send_goal()
        self.assertEqual([r['type'] for r in m.records], ['goal', 'sample'])
        self.assertEqual(m.records[1]['truth']['x'], m.case['start'][0])
        self.assertEqual(len(m.goal_publisher.messages), 1)
        sent = m.goal_publisher.messages[0]
        self.assertEqual(sent.header.frame_id, 'map')
        self.assertEqual([sent.pose.position.x, sent.pose.position.y], m.case['goal'])
        self.assertEqual(m.last_sample_sim, m.sim)
        saved = json.loads((self.directory / 'metadata.json').read_text())
        self.assertEqual(saved['contacts_bridge_sha256'], sha256(self.directory / 'contacts-bridge.yaml'))

    def test_readiness_stationary_interval_resets_on_lost_prerequisite(self):
        m = self.monitor
        m.update_parameters = Mock(return_value=True)
        m.graph_ok = Mock(return_value=True)
        with patch.dict(sys.modules, {'rclpy.time': NS(Time=lambda: object())}):
            self.assertFalse(m.ready())
            self.refresh(1.5)
            self.assertFalse(m.ready())
            self.refresh(1.7)
            m.truth = None
            self.assertFalse(m.ready())
            self.assertIsNone(m.stationary_since)
            self.refresh(2.2)
            self.assertFalse(m.ready())
            self.refresh(3.3)
            self.assertTrue(m.ready())

    def test_fresh_lidar_odom_do_not_hide_wrong_spawn_offset(self):
        m = self.monitor
        m.update_parameters = Mock(return_value=True)
        m.truth['x'] += 0.04
        with patch.dict(sys.modules, {'rclpy.time': NS(Time=lambda: object())}):
            with self.assertRaisesRegex(RuntimeError, 'Actual Gazebo spawn'):
                m.ready()

    def parameter_fixture(self):
        m = self.monitor
        bringup = self.directory / 'bringup'
        (bringup / 'map').mkdir(parents=True)
        (bringup / 'params').mkdir()
        map_yaml = bringup / 'map/rmuc_2026.yaml'
        image = bringup / 'map/rmuc_2026.pgm'
        params = bringup / 'params/no_smoother_tunnels.yaml'
        map_yaml.write_text(json.dumps({'image': image.name}))
        image.write_bytes(b'synthetic fixture, not a PGM test')
        params.write_text('synthetic parameters')
        m.params = {
            '/nav_executor': dict(zip(PARAMETER_NAMES['/nav_executor'],
                                      [False, 0.5, 0.2, 20.0, 0.05, 10, 0.01])),
            '/map_server': dict(zip(PARAMETER_NAMES['/map_server'],
                                    [str(map_yaml), '', 0.3, 0.01, 0.4, False])),
            '/map_client': dict(zip(PARAMETER_NAMES['/map_client'], [0.3, False, False])),
        }
        m.manifest = {'protected_inputs': {
            'nav/map_pgm': {'sha256': sha256(image)},
            'nav/src/bringup/map/rmuc_2026.yaml': {'sha256': sha256(map_yaml)},
            'nav/src/bringup/params/no_smoother_tunnels.yaml': {'sha256': sha256(params)},
        }}
        m.metadata['map_sha256_ok'] = False
        return bringup, {
            'rclpy.parameter': NS(parameter_value_to_python=lambda value: value),
            'ament_index_python.packages': NS(get_package_share_directory=lambda package: str(bringup)),
            'yaml': NS(safe_load=json.loads),
        }

    def test_live_parameters_do_not_query_destroyed_bootstrap_filename(self):
        m = self.monitor
        _, modules = self.parameter_fixture()
        self.assertNotIn('navigation_params_file', PARAMETER_NAMES['/nav_executor'])
        with patch.dict(sys.modules, modules):
            self.assertTrue(m.update_parameters())
        self.assertIn('bootstrap-only', m.metadata['navigation_params_provenance'])
        self.assertEqual(m.metadata['smoother_enabled'], False)

    def test_live_wrong_tuning_and_changed_installed_params_fail(self):
        m = self.monitor
        bringup, modules = self.parameter_fixture()
        with patch.dict(sys.modules, modules):
            m.params['/nav_executor']['MPC.dt'] = 0.1
            with self.assertRaisesRegex(RuntimeError, 'Unexpected live parameter'):
                m.update_parameters()
            m.params['/nav_executor']['MPC.dt'] = 0.05
            (bringup / 'params/no_smoother_tunnels.yaml').write_text('changed')
            with self.assertRaisesRegex(RuntimeError, 'different navigation configuration'):
                m.update_parameters()

    def test_parameter_future_timeout_uses_wall_time(self):
        m = self.monitor
        m.param_pending = {'/nav_executor': NS(done=lambda: False)}
        m.param_started = {'/nav_executor': 1.0}
        m.parameter_clients = {'/nav_executor': object()}
        m.now_wall = 12.0
        with patch.dict(sys.modules, {'rclpy.parameter': NS(parameter_value_to_python=lambda value: value)}):
            with self.assertRaises(TimeoutError):
                m.update_parameters()

    def test_path_digest_ignores_stamp_but_includes_pose_geometry(self):
        a, b = pose_stamped(), pose_stamped()
        a.header.stamp = stamp(1)
        b.header.stamp = stamp(100)
        self.assertEqual(path_digest(NS(poses=[a])), path_digest(NS(poses=[b])))
        b.pose.orientation.z = 0.1
        self.assertNotEqual(path_digest(NS(poses=[a])), path_digest(NS(poses=[b])))
        self.assertIsNone(path_digest(NS(poses=[])))

    def test_full_synthetic_event_recording_replays_as_pass_but_error_overrides(self):
        m = self.monitor
        for entry in passing_records(m.case):
            m.sim = entry['sim_time']
            m.now_wall = entry['wall_time']
            m.record(entry['type'], **{k: v for k, v in entry.items() if k not in ('type', 'sim_time', 'wall_time')})
        m.original_paths, m.output_paths = {'same'}, {'same'}
        self.assertTrue(m.evaluate()['passed'])
        result = m.finish(TimeoutError('synthetic clock stall after hold'))
        self.assertFalse(result['passed'])
        self.assertTrue(m.recording_closed)
        self.assertEqual(m.records[-1]['type'], 'timeout')
        self.assertEqual(result['events_sha256'], sha256(self.directory / 'events.jsonl'))

    def test_fault_injection_requires_current_motion_and_recovery_requires_xy(self):
        m = self.monitor
        m.case.update(kind='sensor_dropout', fault_sensor='imu')
        m.goal_time = 0.0
        self.refresh(1.0, speed=0.3)
        m.update_fault()
        self.refresh(1.4, speed=0.3)
        m.truth['stamp'] = 0.0
        m.update_fault()
        self.assertIsNone(m.fault_begin)
        self.refresh(2.0, speed=0.3)
        m.update_fault()
        self.refresh(2.4, speed=0.3)
        m.update_fault()
        self.assertEqual(m.blocked, {'imu'})
        self.refresh(5.5)
        m.update_fault()
        self.assertFalse(m.blocked)
        self.refresh(5.7, speed=0.3)
        m.truth['z'] += 0.3
        m.update_fault()
        self.assertFalse(m.auto_recovered)
        m.truth['x'] += 0.12
        m.update_fault()
        self.assertTrue(m.auto_recovered)


if __name__ == '__main__':
    unittest.main()
