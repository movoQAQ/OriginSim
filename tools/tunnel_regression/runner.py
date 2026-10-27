"""Portable suite orchestration; ROS is imported only in live execution."""

import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import time
import uuid

from .harness import launch_command, write_spawn_config
from .metrics import aggregate_results, evaluate_trial, expand_cases, load_cases
from .provenance import NAV_ROOT, sha256, source_manifest, verify_install, write_json
from .safety import SENSORS, evaluate_safety


def make_cases(mode='all', tunnels=None):
    cases = expand_cases(load_cases())
    names = {c['tunnel'] for c in cases}
    unknown = set(tunnels or ()) - names
    if unknown:
        raise ValueError('Unknown tunnel(s): ' + ', '.join(sorted(unknown)))
    if mode == 'safety' and tunnels:
        raise ValueError('--tunnel only filters traversal cases; safety uses red_bottom center')
    traversal = [{**case, 'kind': 'traversal'} for case in cases
                 if not tunnels or case['tunnel'] in tunnels] if mode != 'safety' else []
    safety = []
    if mode != 'traversal':
        template = next(c for c in cases if c['tunnel'] == 'red_bottom' and c['trial'] == 1)
        for sensor in SENSORS:
            safety.append({**template, 'case_id': 'dropout-' + sensor,
                           'kind': 'sensor_dropout', 'fault_sensor': sensor})
    return traversal + safety


def suite_summary(plan, results, status, execution_backend='gazebo'):
    expected = [c for c in plan['cases'] if c['kind'] == 'traversal']
    rows = [r for r in results if r.get('kind') == 'traversal']
    report = aggregate_results(rows, expected, execution_backend) if expected else {
        'expected_trials': 0, 'completed_trials': 0, 'passed_trials': 0,
        'all_passed': True, 'rows': [], 'execution_backend': execution_backend}
    safety = [r for r in results if r.get('kind') == 'sensor_dropout']
    wanted = {c['case_id'] for c in plan['cases'] if c['kind'] == 'sensor_dropout'}
    observed = [r.get('case_id') for r in safety]
    safety_ok = (len(observed) == len(wanted) and set(observed) == wanted and
                 all(r.get('passed') is True for r in safety))
    report.update(schema_version=1, suite='.', execution_status=status,
                  generated_at=datetime.now(timezone.utc).isoformat(),
                  scope=plan['scope'], expected_safety_tests=len(wanted),
                  completed_safety_tests=len(safety),
                  passed_safety_tests=sum(r.get('passed') is True for r in safety),
                  safety_results=safety, historical_results_used_as_new_evidence=False,
                  manifest='source-manifest.json', plan='plan.json')
    report['all_passed'] = bool(report['all_passed'] and safety_ok and status == 'completed')
    measured = execution_backend == 'gazebo'
    report['is_gazebo_measurement'] = measured and any(r.get('sample_count', 0) > 0 for r in results)
    report['gazebo_cases_with_recorded_samples'] = sum(
        measured and r.get('sample_count', 0) > 0 for r in results)
    return report


def stop_process(proc):
    """Stop only a session created by this runner, including its descendants."""
    if proc is None:
        return
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        # Reap the leader, but grant descendants the SAME signal grace even if
        # their leader has already exited. proc.wait() alone returns immediately
        # in that case and would escalate to SIGKILL before children flush logs.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
        if sig == signal.SIGKILL:
            break
    try:
        proc.wait(timeout=0.1)
    except subprocess.TimeoutExpired:
        pass


def environment_check(manifest):
    missing = [name for name in ('ros2', 'gz') if shutil.which(name) is None]
    for name in ('rclpy', 'ament_index_python', 'yaml', 'ros_gz_interfaces'):
        if importlib.util.find_spec(name) is None:
            missing.append('Python module ' + name)
    if missing:
        raise RuntimeError('Missing runtime dependencies: ' + ', '.join(missing) +
                           '; source the built Jazzy overlay. No Gazebo trial was run.')
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('This suite targets ROS 2 Jazzy / Gazebo Harmonic; source Jazzy first')
    return verify_install(manifest)


def run_case(case, trial, manifest_path, env, timeouts):
    trial.mkdir()
    write_json(trial / 'case.json', case)
    spawn = write_spawn_config(trial / 'spawn.yaml', *case['start'], case['spawn_z'])
    collector_argv = [sys.executable, str(Path(__file__).with_name('collect.py')),
                      '--trial-dir', str(trial), '--manifest', str(manifest_path),
                      '--startup-timeout', str(timeouts['startup']),
                      '--trial-timeout', str(timeouts['trial']),
                      '--sim-timeout', str(timeouts['sim'])]
    stack_argv = launch_command(spawn, (*case['start'], case['initial_z']))
    trial_env = dict(env, GZ_PARTITION='originsim-regression-' + uuid.uuid4().hex,
                     ROS_LOG_DIR=str(trial / 'ros-log'))
    write_json(trial / 'invocation.json', {
        'collector_argv': collector_argv, 'stack_argv': stack_argv,
        'ROS_DOMAIN_ID': trial_env['ROS_DOMAIN_ID'],
        'GZ_PARTITION': trial_env['GZ_PARTITION'],
        'note': 'Commands are provenance; replay/analysis uses relative case/events/metadata paths.'})
    collector = stack = None
    failure = None
    try:
        with (trial / 'collector.log').open('w') as monitor_log, (trial / 'launch.log').open('w') as stack_log:
            collector = subprocess.Popen(collector_argv, env=trial_env, stdout=monitor_log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
            # The gate/subscribers must exist before startup calibration begins.
            ready_deadline = time.monotonic() + 20
            while not (trial / 'collector-ready.json').exists():
                if collector.poll() is not None:
                    raise RuntimeError('Collector exited before readiness; see collector.log')
                if time.monotonic() > ready_deadline:
                    raise TimeoutError('Collector did not initialize within 20 wall seconds')
                time.sleep(0.05)
            stack = subprocess.Popen(stack_argv, env=trial_env, stdout=stack_log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + timeouts['startup'] + timeouts['trial'] + 10
            while collector.poll() is None:
                if stack.poll() is not None:
                    raise RuntimeError('Simulation/navigation launch exited before collector')
                if time.monotonic() > deadline:
                    raise TimeoutError('Suite supervisor wall deadline exceeded')
                time.sleep(0.1)
            if collector.returncode != 0:
                raise RuntimeError('Collector exited with code ' + str(collector.returncode))
    except KeyboardInterrupt:
        failure = 'interrupted_by_user'
        raise
    except (OSError, RuntimeError, TimeoutError) as exc:
        failure = str(exc)
    finally:
        if failure:
            write_json(trial / 'abort.json', {'failure': failure})
        # Collector unblocks relay and sends teardown zeros before stack shutdown.
        stop_process(collector)
        stop_process(stack)
    result_path = trial / 'result.json'
    row = None
    if result_path.exists():
        try:
            row = json.loads(result_path.read_text(encoding='utf-8'))
            if (not isinstance(row, dict) or row.get('case_id') != case['case_id']
                    or row.get('kind') != case['kind'] or type(row.get('passed')) is not bool
                    or not isinstance(row.get('failures', []), list)):
                raise ValueError('Result does not identify this case with a Boolean verdict')
        except (OSError, ValueError, TypeError) as exc:
            failure = 'invalid_collector_result: ' + str(exc)
            row = None
    if row is None:
        row = {key: case[key] for key in ('case_id', 'kind', 'tunnel', 'trial', 'start_offset_m')}
        row.update(passed=False, failures=[failure or 'collector_produced_no_result'],
                   execution_backend='gazebo', execution_status='error')
    if row.get('passed') is True and not failure:
        try:
            metadata = json.loads((trial / 'metadata.json').read_text(encoding='utf-8'))
            if not isinstance(metadata, dict):
                raise ValueError('Recording metadata is not an object')
            if (metadata.get('recording_complete') is not True or metadata.get('failure')
                    or metadata.get('execution_backend') != 'gazebo'
                    or row.get('sample_count', 0) < 2
                    or row.get('events_sha256') != sha256(trial / 'events.jsonl')):
                raise ValueError('Incomplete recording, wrong backend or event hash mismatch')
        except (OSError, ValueError, TypeError) as exc:
            failure = 'missing_or_inconsistent_pass_evidence: ' + str(exc)
    if failure:
        row['passed'] = False
        row.setdefault('failures', []).append(failure)
    write_json(result_path, row)
    return {**row, 'raw_result': 'trials/' + case['case_id'] + '/result.json',
            'raw_sha256': sha256(result_path),
            'events': 'trials/' + case['case_id'] + '/events.jsonl',
            'events_sha256': sha256(trial / 'events.jsonl') if (trial / 'events.jsonl').exists() else None}


def read_records(path):
    records = []
    with Path(path).open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            record = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(record, dict):
                raise ValueError('Not an event object at line ' + str(line_number))
            records.append(record)
    return records


def analyze(directory):
    directory = Path(directory).resolve()
    plan = json.loads((directory / 'plan.json').read_text(encoding='utf-8'))
    rows, errors, backends = [], [], set()
    for case in plan['cases']:
        trial = directory / 'trials' / case['case_id']
        # Case identifiers must remain relative even when consuming another run.
        if not trial.resolve().is_relative_to(directory / 'trials'):
            raise ValueError('Case path escapes the run directory')
        try:
            metadata = json.loads((trial / 'metadata.json').read_text(encoding='utf-8'))
            if not isinstance(metadata, dict):
                raise ValueError('Recording metadata is not an object')
            backend = metadata.get('execution_backend')
            if backend not in ('gazebo', 'synthetic'):
                raise ValueError('Recording must identify its execution_backend')
            records = read_records(trial / 'events.jsonl')
            row = evaluate_trial(case, records, metadata)
            if case['kind'] == 'sensor_dropout':
                row = evaluate_safety(case, records, row)
            row.update(case_id=case['case_id'], kind=case['kind'], execution_backend=backend)
            backends.add(backend)
            if metadata.get('failure') or not metadata.get('recording_complete'):
                row['passed'] = False
                row.setdefault('failures', []).append('incomplete_or_failed_recording')
            original_path = trial / 'result.json'
            if original_path.exists():
                original = json.loads(original_path.read_text(encoding='utf-8'))
                if not isinstance(original, dict):
                    raise ValueError('Original result is not an object')
                if original.get('events_sha256') != sha256(trial / 'events.jsonl'):
                    row['passed'] = False
                    row.setdefault('failures', []).append('event_file_hash_mismatch')
            row['raw_result'] = 'trials/' + case['case_id'] + '/result.json'
            rows.append(row)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # Missing/unreadable files are not completed physical trials.
            errors.append({'case_id': case['case_id'], 'error': 'cannot_replay: ' + str(exc)})
    backend = next(iter(backends)) if len(backends) == 1 else ('mixed' if backends else 'none')
    report = suite_summary(plan, rows, 'completed', backend)
    report.update(execution_method='offline_replay', new_gazebo_trials=0,
                  analysis_errors=errors, analysis_metrics_sha256=sha256(Path(__file__).with_name('metrics.py')),
                  analysis_safety_sha256=sha256(Path(__file__).with_name('safety.py')))
    if errors or len(backends) != 1:
        report['all_passed'] = False
    write_json(directory / 'analysis.json', report)
    return 0 if report['all_passed'] else 1


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    sub = root.add_subparsers(dest='command', required=True)
    for name in ('plan', 'run'):
        cmd = sub.add_parser(name, help='Prepare without ROS' if name == 'plan' else 'Execute fresh Gazebo trials')
        cmd.add_argument('--nav-root', type=Path, default=NAV_ROOT)
        cmd.add_argument('--gazebo-root', type=Path, default=NAV_ROOT.parent / 'OriginSim-gazebo')
        cmd.add_argument('--output', type=Path, required=True, help='New, empty output directory')
        cmd.add_argument('--mode', choices=('all', 'traversal', 'safety'), default='all')
        cmd.add_argument('--tunnel', action='append', help='Repeat to filter traversal tunnels')
        if name == 'run':
            cmd.add_argument('--startup-timeout', type=float, default=180)
            cmd.add_argument('--trial-timeout', type=float, default=240)
            cmd.add_argument('--sim-timeout', type=float, default=90)
            cmd.add_argument('--ros-domain-id', type=int, default=None,
                             help='Empty, dedicated DDS domain (default: random 20..89)')
    replay = sub.add_parser('analyze', help='Recompute analysis.json from saved JSONL; launches nothing')
    replay.add_argument('run_directory', type=Path)
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == 'analyze':
        return analyze(args.run_directory)
    cases = make_cases(args.mode, args.tunnel)
    output = args.output.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('Refusing to overwrite a nonempty output directory: ' + str(output))
    manifest = source_manifest(args.nav_root, args.gazebo_root)
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / 'source-manifest.json', manifest)
    plan = {'schema_version': 1, 'cases': cases, 'mode': args.mode,
            'execution_status': 'planned', 'scope': {
                'directions': 'blue north-to-south; red west-to-east',
                'start': 'stationary calibrated yaw=0, prescribed positions',
                'lidar': '10 Hz, 720 x 32', 'map': 'original rmuc_2026 PGM',
                'core_intervals': 'rederived from PGM; historical private script unavailable',
                'does_not_cover': ['reverse travel', 'arbitrary starts', '100 Hz LiDAR', 'real robot']}}
    write_json(output / 'plan.json', plan)
    if args.command == 'plan':
        print(json.dumps({'status': 'planned_only', 'cases': len(cases),
                          'output': str(output), 'gazebo_trials_executed': 0}, ensure_ascii=False))
        return 0
    for name in ('startup_timeout', 'trial_timeout', 'sim_timeout'):
        value = getattr(args, name)
        if not 0 < value <= 3600:
            raise ValueError('Timeouts must be finite positive values <=3600 seconds')
    domain = args.ros_domain_id if args.ros_domain_id is not None else 20 + secrets.randbelow(70)
    if not 0 <= domain <= 101:
        raise ValueError('Use an isolated ROS domain in [0,101]')
    results = []
    try:
        installed = environment_check(manifest)
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        report = suite_summary(plan, [], 'blocked')
        report['failure'] = str(exc)
        write_json(output / 'results.json', report)
        print(str(exc), file=sys.stderr)
        return 2
    write_json(output / 'installed-manifest.json', installed)
    (output / 'trials').mkdir()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               PYTHONUNBUFFERED='1')
    status = 'running'
    write_json(output / 'results.json', suite_summary(plan, [], status))
    try:
        for number, case in enumerate(cases, 1):
            print(f"[{number}/{len(cases)}] {case['case_id']}", flush=True)
            row = run_case(case, output / 'trials' / case['case_id'],
                           output / 'source-manifest.json', env,
                           {'startup': args.startup_timeout, 'trial': args.trial_timeout,
                            'sim': args.sim_timeout})
            results.append(row)
            print('  ' + ('PASS' if row['passed'] else 'FAIL'), flush=True)
            write_json(output / 'results.json', suite_summary(plan, results, status))
        status = 'completed'
    except KeyboardInterrupt:
        status = 'interrupted'
    finally:
        report = suite_summary(plan, results, status)
        write_json(output / 'results.json', report)
    return 0 if report['all_passed'] else (130 if status == 'interrupted' else 1)
