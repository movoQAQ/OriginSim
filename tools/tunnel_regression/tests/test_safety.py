"""Synthetic fault records exercise safety failures; these are not Gazebo runs."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tunnel_regression.safety import COMMAND_DEADLINES, SENSORS, evaluate_safety


def evidence(sensor='imu'):
    """Fresh live streams, isolated fault, sustained stop, then measured motion."""
    start, end, recovered = 2.0, 5.1, 6.1
    counters = {name: 100 for name in SENSORS}
    records = [{'type': 'goal', 'sim_time': 1.0, 'wall_time': 1.0}]

    def sample(t, after=False):
        during = start < t < end
        command_speed = 0.3 if not during or t < start + COMMAND_DEADLINES[sensor] else 0.0
        truth_speed = 0.3 if not during or t < start + 0.6 else 0.0
        odom_time = min(t, start + (0.6 if sensor == 'lidar' else 0.2)) if during else t
        x = 0.2 + max(0.0, t-end) * 0.3 if after else 0.2
        streams = {'source': {}, 'input': {}}
        forwarded = dict(counters)
        for name in SENSORS:
            count = 100 + max(0, round((t - start) * 100))
            live = {'stamp': t, 'received_sim': t, 'received_wall': t, 'count': count}
            streams['source'][name] = dict(live)
            streams['input'][name] = dict(live)
            if during and name == sensor:
                streams['input'][name] = {'stamp': 1.95, 'received_sim': 1.95,
                                         'received_wall': 1.95, 'count': 100}
            else:
                forwarded[name] = count
        item = {
            'type': 'sample', 'sim_time': t, 'wall_time': t,
            'truth': {'stamp': t, 'received_wall': t, 'x': x, 'y': 0.0, 'z': 0.3,
                      'linear': [truth_speed, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]},
            'command': {'received_sim': t, 'received_wall': t,
                        'linear': [command_speed, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]},
            'odom': {'stamp': odom_time, 'received_sim': odom_time, 'received_wall': odom_time},
            'sensors': streams, 'forwarded_counts': forwarded,
        }
        records.append(item)
        records.append({'type': 'command', 'sim_time': t, 'wall_time': t,
                        'linear': list(item['command']['linear']),
                        'angular': list(item['command']['angular'])})

    sample(1.95)
    records.append({'type': 'fault_started', 'sim_time': start, 'wall_time': start,
                    'sensor': sensor, 'source_counts': dict(counters),
                    'forwarded_counts': dict(counters)})
    for index in range(1, 31):
        sample(round(start + index * 0.1, 3))
    records.append({'type': 'fault_ended', 'sim_time': end, 'wall_time': end,
                    'sensor': sensor, 'source_counts': {name: 410 for name in SENSORS},
                    'forwarded_counts': {name: 100 if name == sensor else 410 for name in SENSORS}})
    for index in range(1, 11):
        sample(round(end + index * 0.1, 3), after=True)
    records.append({'type': 'auto_recovered', 'sim_time': recovered,
                    'wall_time': recovered, 'displacement_m': 0.3})
    return {'fault_sensor': sensor, 'case_id': 'dropout-' + sensor}, records


class SafetyEvidenceTest(unittest.TestCase):
    def result(self, case, records, traversal_passed=True):
        return evaluate_safety(case, records, {'passed': traversal_passed})

    def reject(self, records, reason, sensor='imu'):
        result = self.result({'fault_sensor': sensor}, records)
        self.assertFalse(result['passed'])
        self.assertIn(reason, result['sensor_dropout_test']['failures'])

    def test_independent_lidar_imu_and_wheel_faults(self):
        for sensor in SENSORS:
            with self.subTest(sensor=sensor):
                case, records = evidence(sensor)
                result = self.result(case, records)
                self.assertTrue(result['passed'], result['sensor_dropout_test'])
                self.assertGreaterEqual(
                    result['sensor_dropout_test']['measured_recovery_displacement_m'], 0.1)
                self.assertTrue(result['sensor_dropout_test']['suppression_observed'])

    def test_continuing_odometry_fails_even_if_commands_and_robot_stop(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 2 < record['sim_time'] < 5.1:
                record['odom']['stamp'] = record['sim_time']
                record['odom']['received_wall'] = record['wall_time']
        self.reject(records, 'sensor_fault_did_not_suppress_odom_stamp')

    def test_republished_old_odometry_stamp_fails_receipt_suppression(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 2 < record['sim_time'] < 5.1:
                record['odom']['received_wall'] = record['wall_time']
        self.reject(records, 'sensor_fault_did_not_suppress_odom_receipt')

    def test_heave_alone_cannot_prove_planar_recovery(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and record['sim_time'] > 5.1:
                record['truth']['x'] = 0.2
                record['truth']['z'] += record['sim_time']-5.1
                record['truth']['linear'] = [0, 0, 0.3]
        self.reject(records, 'recovery_requires_fresh_input_odom_and_actual_motion')

    def test_vertical_velocity_still_invalidates_physical_stop(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 4 <= record['sim_time'] < 5.1:
                record['truth']['linear'][2] = 0.1
        self.reject(records, 'robot_still_moving_after_physical_stop_deadline')

    def test_nonzero_command_pulse_between_samples_fails(self):
        _, records = evidence()
        records.append({'type': 'command', 'sim_time': 4.04, 'wall_time': 4.04,
                        'linear': [0.01, 0, 0], 'angular': [0, 0, 0]})
        records.sort(key=lambda r: r['sim_time'])
        self.reject(records, 'nonzero_command_after_stop_deadline')

    def test_missing_actual_selected_input_fails(self):
        _, records = evidence()
        before = next(r for r in records if r['type'] == 'sample')
        before['sensors']['input']['imu']['count'] = 0
        self.reject(records, 'selected_input_not_observed_live_before_fault')

    def test_source_stopped_with_gate_fails(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 2 < record['sim_time'] < 5.1:
                record['sensors']['source']['imu']['stamp'] = 1.9
        self.reject(records, 'source_stale_imu')

    def test_whole_bridge_freeze_cannot_pass_independent_fault(self):
        _, records = evidence()
        end = next(r for r in records if r['type'] == 'fault_ended')
        end['source_counts'] = {name: 100 for name in SENSORS}
        self.reject(records, 'source_did_not_continue_lidar')

    def test_nonselected_input_is_required_to_continue(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 2 < record['sim_time'] < 5.1:
                record['sensors']['input']['wheels']['stamp'] = 1.9
        self.reject(records, 'unaffected_input_stale_wheels')

    def test_gate_that_keeps_forwarding_fails(self):
        _, records = evidence()
        sample = next(r for r in records if r['type'] == 'sample' and r['sim_time'] == 4)
        sample['forwarded_counts']['imu'] += 1
        self.reject(records, 'selected_gate_forwarded_during_fault')

    def test_replayed_input_behind_closed_gate_fails(self):
        _, records = evidence()
        sample = next(r for r in records if r['type'] == 'sample' and r['sim_time'] == 4)
        sample['sensors']['input']['imu']['received_sim'] = 4
        sample['sensors']['input']['imu']['received_wall'] = 4
        self.reject(records, 'selected_input_still_delivered')

    def test_missing_command_events_cannot_mean_safe_zero(self):
        _, records = evidence()
        records = [r for r in records if r['type'] != 'command']
        self.reject(records, 'missing_post_deadline_commands')

    def test_missing_truth_fails(self):
        _, records = evidence()
        sample = next(r for r in records if r['type'] == 'sample' and r['sim_time'] == 4)
        sample['truth']['stamp'] = 1.9
        self.reject(records, 'truth_lost_during_fault')

    def test_zero_velocity_with_drifting_truth_position_fails(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and 4 <= record['sim_time'] < 5.1:
                record['truth']['x'] += record['sim_time'] - 4
        self.reject(records, 'truth_position_drift_during_stop')

    def test_resumed_commands_without_measured_translation_fails(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and record['sim_time'] > 5.1:
                record['truth']['x'] = 0.2
        self.reject(records, 'recovery_truth_displacement_not_observed')

    def test_recovery_with_stale_odometry_fails(self):
        _, records = evidence()
        for record in records:
            if record['type'] == 'sample' and record['sim_time'] > 5.1:
                record['odom']['stamp'] = 2.0
        self.reject(records, 'recovery_requires_fresh_input_odom_and_actual_motion')

    def test_early_stop_sampling_before_restore_fails(self):
        _, records = evidence()
        records = [r for r in records if not(r['type'] == 'sample' and 4.5 <= r['sim_time'] < 5.1)]
        self.reject(records, 'fault_observation_missing_at_boundary')

    def test_stationary_fault_does_not_count_as_safe_stop(self):
        _, records = evidence()
        before = next(r for r in records if r['type'] == 'sample')
        before['truth']['linear'] = [0, 0, 0]
        self.reject(records, 'fault_not_injected_during_actual_translation')

    def test_a_second_goal_is_not_automatic_recovery(self):
        _, records = evidence()
        records.append({'type': 'goal', 'sim_time': 5.2, 'wall_time': 5.2})
        self.reject(records, 'exactly_one_original_goal_required')

    def test_safe_stop_does_not_override_failed_traversal(self):
        case, records = evidence()
        result = self.result(case, records, traversal_passed=False)
        self.assertFalse(result['passed'])
        self.assertIn('original_goal_did_not_finish_all_traversal_checks',
                      result['sensor_dropout_test']['failures'])


if __name__ == '__main__':
    unittest.main()
