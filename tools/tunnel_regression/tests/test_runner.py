"""Subprocess and suite-state simulations; no ROS or Gazebo execution implied."""

import json
from pathlib import Path
import signal
import sys
import tempfile
from unittest import mock
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tunnel_regression import runner
from test_metrics import METADATA, passing_records


class RunnerSupervisionTest(unittest.TestCase):
    def test_all_case_plan_has_fifteen_traversals_and_three_independent_faults(self):
        cases = runner.make_cases()
        self.assertEqual(sum(c['kind'] == 'traversal' for c in cases), 15)
        self.assertEqual([c['fault_sensor'] for c in cases if c['kind'] == 'sensor_dropout'],
                         ['lidar', 'imu', 'wheels'])
        self.assertEqual(len({c['case_id'] for c in cases}), 18)

    def test_partial_or_duplicated_fault_suite_cannot_pass(self):
        cases = runner.make_cases('safety')
        plan = {'cases': cases, 'scope': {}}
        rows = [{'case_id': case['case_id'], 'kind': 'sensor_dropout', 'passed': True}
                for case in cases]
        self.assertTrue(runner.suite_summary(plan, rows, 'completed')['all_passed'])
        self.assertFalse(runner.suite_summary(plan, rows[:2], 'completed')['all_passed'])
        self.assertFalse(runner.suite_summary(plan, [rows[0], rows[0], rows[2]],
                                            'completed')['all_passed'])
        self.assertFalse(runner.suite_summary(plan, rows, 'interrupted')['all_passed'])

    def simulate_case(self, collector_status=0, stack_status=None, write_result=True,
                      result_override=None):
        case = runner.make_cases('traversal', ['upper'])[0]
        with tempfile.TemporaryDirectory() as directory:
            trial = Path(directory) / case['case_id']
            collector = mock.Mock(pid=1001, returncode=collector_status)
            collector.poll.return_value = collector_status
            stack = mock.Mock(pid=1002, returncode=stack_status)
            stack.poll.return_value = stack_status

            def spawn(argv, **kwargs):
                self.assertTrue(kwargs['start_new_session'])
                self.assertFalse(kwargs.get('shell', False))
                if 'collect.py' in argv[1]:
                    (trial / 'collector-ready.json').write_text('{}')
                    if write_result:
                        result = json.dumps({
                            **{k: case[k] for k in ('case_id', 'kind', 'tunnel', 'trial', 'start_offset_m')},
                            'passed': True,
                        })
                        (trial / 'result.json').write_text(result if result_override is None
                                                          else result_override)
                    return collector
                return stack

            with mock.patch.object(runner.subprocess, 'Popen', side_effect=spawn), \
                 mock.patch.object(runner, 'stop_process') as stop:
                row = runner.run_case(case, trial, Path(directory)/'manifest.json',
                                      {'ROS_DOMAIN_ID': '79'},
                                      {'startup': 2, 'trial': 2, 'sim': 2})
            self.assertEqual(stop.call_args_list, [mock.call(collector), mock.call(stack)])
            self.assertEqual(row['raw_result'], 'trials/' + case['case_id'] + '/result.json')
            self.assertTrue(row['raw_sha256'])
            return row

    def test_nonzero_collector_exit_cannot_leave_a_passing_result(self):
        row = self.simulate_case(collector_status=1)
        self.assertFalse(row['passed'])
        self.assertTrue(any('Collector exited with code 1' in message for message in row['failures']))

    def test_zero_exit_without_raw_evidence_cannot_pass(self):
        row = self.simulate_case(collector_status=0)
        self.assertFalse(row['passed'])
        self.assertTrue(any('missing_or_inconsistent_pass_evidence' in message
                            for message in row['failures']))

    def test_missing_archive_is_not_counted_as_executed_gazebo_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'plan.json').write_text(json.dumps({'cases': runner.make_cases(), 'scope': {}}))
            self.assertEqual(runner.analyze(root), 1)
            report = json.loads((root / 'analysis.json').read_text())
            self.assertEqual(report['completed_trials'], 0)
            self.assertEqual(report['completed_safety_tests'], 0)
            self.assertFalse(report['is_gazebo_measurement'])
            self.assertEqual(len(report['analysis_errors']), 18)

    def test_synthetic_archive_replay_remains_synthetic_and_hash_changes_fail(self):
        case = runner.make_cases('traversal', ['upper'])[0]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trial = root / 'trials' / case['case_id']
            trial.mkdir(parents=True)
            (root / 'plan.json').write_text(json.dumps({'cases': [case], 'scope': {}}))
            records = passing_records(case)
            events = trial / 'events.jsonl'
            events.write_text(''.join(json.dumps(row) + '\n' for row in records))
            (trial / 'metadata.json').write_text(json.dumps({**METADATA,
                'recording_complete': True, 'execution_backend': 'synthetic'}))
            (trial / 'result.json').write_text(json.dumps({'events_sha256': runner.sha256(events)}))
            self.assertEqual(runner.analyze(root), 0)
            report = json.loads((root / 'analysis.json').read_text())
            self.assertEqual(report['execution_backend'], 'synthetic')
            self.assertFalse(report['is_gazebo_measurement'])
            self.assertEqual(report['new_gazebo_trials'], 0)
            with events.open('a') as stream:
                stream.write('\n')
            self.assertEqual(runner.analyze(root), 1)
            report = json.loads((root / 'analysis.json').read_text())
            self.assertIn('event_file_hash_mismatch', report['rows'][0]['failures'])

    def test_missing_result_cannot_pass(self):
        row = self.simulate_case(write_result=False)
        self.assertFalse(row['passed'])
        self.assertIn('collector_produced_no_result', row['failures'])

    def test_truncated_or_wrong_case_result_is_reported_as_failure(self):
        for result in ('{"passed":', '[true]',
                       '{"passed":true,"case_id":"wrong","kind":"traversal"}'):
            with self.subTest(result=result):
                row = self.simulate_case(result_override=result)
                self.assertFalse(row['passed'])
                self.assertTrue(any('invalid_collector_result' in message
                                    for message in row['failures']))

    def test_stack_exit_aborts_even_if_partial_result_claims_pass(self):
        row = self.simulate_case(collector_status=None, stack_status=1)
        self.assertFalse(row['passed'])
        self.assertTrue(any('launch exited before collector' in message for message in row['failures']))

    def test_exited_leader_does_not_remove_descendants_signal_grace(self):
        proc = mock.Mock(pid=12345)
        proc.poll.return_value = 0
        clock = [0.0]
        sent = []

        def killpg(pid, sig):
            self.assertEqual(pid, proc.pid)
            if sig:
                sent.append(sig)
            elif clock[0] >= 0.2:
                raise ProcessLookupError

        def advance(seconds):
            clock[0] += seconds

        with mock.patch.object(runner.os, 'killpg', side_effect=killpg), \
             mock.patch.object(runner.time, 'monotonic', side_effect=lambda: clock[0]), \
             mock.patch.object(runner.time, 'sleep', side_effect=advance):
            runner.stop_process(proc)
        self.assertGreaterEqual(clock[0], 0.2)
        self.assertEqual(sent, [signal.SIGINT], 'Child must get time to flush on SIGINT')

    def test_ignoring_group_is_escalated_with_bounded_grace(self):
        proc = mock.Mock(pid=12345)
        clock = [0.0]
        sent = []

        def killpg(pid, sig):
            self.assertEqual(pid, proc.pid)
            if sig:
                sent.append(sig)

        with mock.patch.object(runner.os, 'killpg', side_effect=killpg), \
             mock.patch.object(runner.time, 'monotonic', side_effect=lambda: clock[0]), \
             mock.patch.object(runner.time, 'sleep', side_effect=lambda s: clock.__setitem__(0, clock[0]+s)):
            runner.stop_process(proc)
        self.assertEqual(sent, [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])
        self.assertLess(clock[0], 10.2)


if __name__ == '__main__':
    unittest.main()
