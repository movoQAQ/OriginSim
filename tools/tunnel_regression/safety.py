"""Pure, replayable checks for one independently interrupted sensor stream."""

import math


SENSORS = ('lidar', 'imu', 'wheels')
# Existing LiDAR freshness is 0.5 s, wheel/IMU 0.1 s, executor timeout 0.5 s.
# Deadlines include scheduling slack, without changing those production values.
COMMAND_DEADLINES = {'lidar': 1.3, 'imu': 0.9, 'wheels': 0.9}
# The existing backend suppresses output after 0.5 sim/0.8 wall seconds for
# LiDAR, and 0.1 sim/0.3 wall seconds for wheel/IMU freshness. The acceptance
# deadlines below add observation/scheduling slack without changing the node.
ODOMETRY_STAMP_DEADLINES = {'lidar': 0.7, 'imu': 0.3, 'wheels': 0.3}
ODOMETRY_RECEIPT_DEADLINES = {'lidar': 1.0, 'imu': 0.5, 'wheels': 0.5}
HOLD_SIM_SECONDS = 3.0
HOLD_WALL_SECONDS = 1.0
STOP_DEADLINE_SECONDS = 2.0
RECOVERY_DEADLINE_SECONDS = 15.0
MAX_STOP_DRIFT_METRES = 0.05


def magnitude(vector):
    if len(vector) != 3 or not all(math.isfinite(float(x)) for x in vector):
        raise ValueError('Expected a finite 3-vector')
    return math.sqrt(sum(float(x) ** 2 for x in vector))


def moving(sample):
    try:
        truth, command = sample['truth']['linear'], sample['command']['linear']
        magnitude(truth)
        magnitude(command)
        # Heave while falling into a depression is not resumed planar driving.
        return math.hypot(*truth[:2]) > 0.1 and math.hypot(*command[:2]) > 0.1
    except (KeyError, TypeError, ValueError):
        return False


def position(sample):
    coordinates = [sample['truth'][name] for name in ('x', 'y')]
    if not all(math.isfinite(float(value)) for value in coordinates):
        raise ValueError('Expected finite truth coordinates')
    return coordinates


def _fresh(stream, sample, sim_age=0.35, wall_age=5.0):
    try:
        sa = sample['sim_time'] - stream['stamp']
        wa = sample['wall_time'] - stream['received_wall']
        return math.isfinite(sa) and math.isfinite(wa) and -0.05 <= sa <= sim_age and 0 <= wa <= wall_age
    except (KeyError, TypeError):
        return False


def evaluate_safety(case, records, traversal_result):
    """Require genuine motion, one isolated fault, safe stop and auto completion.

    A gate counter by itself is insufficient: source streams, unaffected
    delivered streams, truth, clock, command output and recovered odometry all
    have independent observations. Every command event is checked, including
    short nonzero pulses between sampled frames.
    """
    row = dict(traversal_result)
    failures = []
    sensor = case.get('fault_sensor')
    starts = [r for r in records if r.get('type') == 'fault_started']
    ends = [r for r in records if r.get('type') == 'fault_ended']
    recovered = [r for r in records if r.get('type') == 'auto_recovered']
    goal_events = [r for r in records if r.get('type') == 'goal']
    result = {'sensor': sensor, 'passed': False, 'failures': failures,
              'command_deadline_s': COMMAND_DEADLINES.get(sensor),
              'odometry_stamp_deadline_s': ODOMETRY_STAMP_DEADLINES.get(sensor),
              'odometry_receipt_wall_deadline_s': ODOMETRY_RECEIPT_DEADLINES.get(sensor),
              'physical_stop_deadline_s': STOP_DEADLINE_SECONDS,
              'minimum_hold_s': HOLD_SIM_SECONDS,
              'test': 'Drop one navigation input; leave sources, truth, clock and other inputs live.'}
    row.update(kind='sensor_dropout', fault_sensor=sensor, sensor_dropout_test=result)

    def require(ok, reason):
        if not ok and reason not in failures:
            failures.append(reason)

    require(sensor in SENSORS, 'unknown_sensor')
    require(len(starts) == 1 and len(ends) == 1, 'one_fault_and_one_restore_required')
    require(len(goal_events) == 1, 'exactly_one_original_goal_required')
    if failures:
        row['passed'] = False
        return row
    begin, end = starts[0], ends[0]
    try:
        start_sim, start_wall = float(begin['sim_time']), float(begin['wall_time'])
        end_sim, end_wall = float(end['sim_time']), float(end['wall_time'])
        require(all(math.isfinite(value) for value in
                    (start_sim, start_wall, end_sim, end_wall)), 'invalid_fault_times')
        require(begin['sensor'] == end['sensor'] == sensor, 'fault_sensor_mismatch')
        require(end_sim - start_sim >= HOLD_SIM_SECONDS - 1e-6, 'fault_hold_too_short_sim')
        require(end_wall - start_wall >= HOLD_WALL_SECONDS - 1e-6, 'fault_hold_too_short_wall')
        result.update(fault_sim_time=start_sim, restore_sim_time=end_sim,
                      simulation_advanced_s=end_sim - start_sim,
                      wall_hold_s=end_wall - start_wall)
        samples = [r for r in records if r.get('type') == 'sample']
        before = [r for r in samples if r['sim_time'] <= start_sim]
        require(bool(before) and start_sim - before[-1]['sim_time'] <= 0.25
                and moving(before[-1]), 'fault_not_injected_during_actual_translation')
        if before:
            selected = before[-1]['sensors']['input'][sensor]
            require(_fresh(selected, before[-1]) and selected.get('count', 0) > 0,
                    'selected_input_not_observed_live_before_fault')
            require(_fresh(before[-1]['odom'], before[-1], 0.5, 2),
                    'odometry_not_live_before_fault')
        during = [r for r in samples if start_sim < r['sim_time'] < end_sim]
        require(len(during) >= 10, 'insufficient_fault_observations')
        if during:
            require(during[0]['sim_time'] - start_sim <= 0.25 and
                    end_sim - during[-1]['sim_time'] <= 0.25 and
                    0 <= during[0]['wall_time'] - start_wall <= 2.0 and
                    0 <= end_wall - during[-1]['wall_time'] <= 2.0,
                    'fault_observation_missing_at_boundary')
        forwarded = begin['forwarded_counts']
        for sample in during:
            require(sample['forwarded_counts'][sensor] == forwarded[sensor],
                    'selected_gate_forwarded_during_fault')
            require(_fresh(sample['truth'], sample, 0.25, 2), 'truth_lost_during_fault')
            for name in SENSORS:
                require(_fresh(sample['sensors']['source'][name], sample),
                        'source_stale_' + name)
                if name != sensor:
                    require(_fresh(sample['sensors']['input'][name], sample),
                            'unaffected_input_stale_' + name)
            # A short DDS drain is allowed, but no delayed/replayed input is.
            delivered = sample['sensors']['input'][sensor]
            if sample['sim_time'] - start_sim > 0.25 and sample['wall_time'] - start_wall > 0.5:
                require(delivered['received_sim'] <= start_sim + 0.25 and
                        delivered['received_wall'] <= start_wall + 0.5,
                        'selected_input_still_delivered')
        for name in SENSORS:
            require(end['source_counts'][name] > begin['source_counts'][name],
                    'source_did_not_continue_' + name)
            if name != sensor:
                require(end['forwarded_counts'][name] > forwarded[name],
                        'other_gate_not_forwarding_' + name)
        require(end['forwarded_counts'][sensor] == forwarded[sensor],
                'selected_gate_did_not_remain_closed')

        # Retain the last actual message while the stream is silent. Explicitly
        # prove output suppression; a stopped command alone cannot establish it.
        odometry = [r['odom'] for r in during]
        observed_odom = ([before[-1]['odom']] if before else []) + odometry
        valid_odom = all(isinstance(item, dict) and
                         math.isfinite(float(item['stamp'])) and
                         math.isfinite(float(item['received_wall']))
                         for item in observed_odom)
        require(bool(observed_odom) and valid_odom, 'missing_odometry_observation_during_fault')
        late_odom = [r for r in during
                     if r['sim_time'] - start_sim > ODOMETRY_STAMP_DEADLINES[sensor]
                     and r['wall_time'] - start_wall > ODOMETRY_RECEIPT_DEADLINES[sensor]]
        require(bool(late_odom), 'no_post_deadline_odometry_suppression_observation')
        if observed_odom and valid_odom:
            last_stamp = max(float(item['stamp']) for item in observed_odom)
            last_receipt = max(float(item['received_wall']) for item in observed_odom)
            stamp_delay, receipt_delay = last_stamp-start_sim, last_receipt-start_wall
            stamp_suppressed = stamp_delay <= ODOMETRY_STAMP_DEADLINES[sensor] + 1e-9
            receipt_suppressed = receipt_delay <= ODOMETRY_RECEIPT_DEADLINES[sensor] + 1e-9
            require(stamp_suppressed, 'sensor_fault_did_not_suppress_odom_stamp')
            require(receipt_suppressed, 'sensor_fault_did_not_suppress_odom_receipt')
            result.update(last_odom_stamp_after_fault_s=stamp_delay if stamp_delay >= 0 else None,
                          last_odom_receipt_after_fault_wall_s=receipt_delay if receipt_delay >= 0 else None,
                          suppression_observed=bool(late_odom) and stamp_suppressed and receipt_suppressed)

        deadline = COMMAND_DEADLINES[sensor]
        commands = [r for r in records if r.get('type') == 'command'
                    and start_sim <= r['sim_time'] < end_sim
                    and (r['sim_time'] - start_sim >= deadline or
                         r['wall_time'] - start_wall >= 2.0)]
        require(len(commands) >= 5, 'missing_post_deadline_commands')
        values = [max(magnitude(r['linear']), magnitude(r['angular'])) for r in commands]
        require(bool(values) and max(values) <= 1e-6, 'nonzero_command_after_stop_deadline')
        result['commands_checked_after_deadline'] = len(commands)
        result['max_command_after_deadline'] = max(values, default=None)
        # Missing commands, rather than explicit zeros, must not count as safe.
        stopped_samples = [r for r in during if r['sim_time'] - start_sim >= deadline]
        for sample in stopped_samples:
            command = sample['command']
            require(-0.05 <= sample['sim_time'] - command['received_sim'] <= 0.25
                    and 0 <= sample['wall_time'] - command['received_wall'] <= 2.0,
                    'command_stream_stale_during_fault')
            require(max(magnitude(command['linear']), magnitude(command['angular'])) <= 1e-6,
                    'nonzero_sampled_command_during_fault')
        physically_stopped = [r for r in during
                              if r['sim_time'] - start_sim >= STOP_DEADLINE_SECONDS]
        require(bool(physically_stopped), 'missing_physical_stop_observations')
        for sample in physically_stopped:
            require(magnitude(sample['truth']['linear']) < 0.05 and
                    magnitude(sample['truth']['angular']) < 0.05,
                    'robot_still_moving_after_physical_stop_deadline')
        if physically_stopped:
            require(end_sim - physically_stopped[0]['sim_time'] >= 0.9,
                    'insufficient_sustained_physical_stop')
            origin = position(physically_stopped[0])
            drift = max(math.dist(origin, position(sample)) for sample in physically_stopped)
            require(drift <= MAX_STOP_DRIFT_METRES, 'truth_position_drift_during_stop')
            result['maximum_stopped_displacement_m'] = drift
        gaps = [(b['sim_time'] - a['sim_time'], b['wall_time'] - a['wall_time'])
                for a, b in zip(during, during[1:])]
        require(bool(gaps) and all(0 < ds <= 0.25 and 0 <= dw <= 2.0 for ds, dw in gaps),
                'fault_observation_gap_or_frozen_clock')

        require(len(recovered) == 1, 'automatic_recovery_not_observed')
        if len(recovered) == 1:
            resume = recovered[0]
            require(0 < resume['sim_time'] - end_sim <= RECOVERY_DEADLINE_SECONDS,
                    'automatic_recovery_sim_timeout')
            require(0 < resume['wall_time'] - end_wall <= 60,
                    'automatic_recovery_wall_timeout')
            after = [r for r in samples if end_sim < r['sim_time'] <= resume['sim_time'] + 0.05]
            evidence = [r for r in after if moving(r) and
                        r['odom']['stamp'] > end_sim and _fresh(r['odom'], r, 0.5, 2)
                        and r['sensors']['input'][sensor]['stamp'] > end_sim
                        and _fresh(r['sensors']['input'][sensor], r)]
            require(bool(evidence), 'recovery_requires_fresh_input_odom_and_actual_motion')
            require(resume.get('displacement_m', 0) >= 0.1,
                    'recovery_requires_measured_displacement')
            measured_displacement = 0.0
            if during and evidence:
                restored_position = position(during[-1])
                measured_displacement = max(math.dist(restored_position, position(sample))
                                            for sample in evidence)
                require(measured_displacement >= 0.1,
                        'recovery_truth_displacement_not_observed')
                result['measured_recovery_displacement_m'] = measured_displacement
            result.update(automatic_recovery=bool(evidence and measured_displacement >= 0.1
                          and resume.get('displacement_m', 0) >= 0.1
                          and 0 < resume['sim_time'] - end_sim <= RECOVERY_DEADLINE_SECONDS
                          and 0 < resume['wall_time'] - end_wall <= 60),
                          recovery_delay_s=resume['sim_time'] - end_sim)
        require(traversal_result.get('passed') is True, 'original_goal_did_not_finish_all_traversal_checks')
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        require(False, 'malformed_fault_evidence: ' + str(exc))
    result['passed'] = not failures
    row['passed'] = result['passed'] and traversal_result.get('passed') is True
    return row
