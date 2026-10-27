"""ROS-independent, fail-closed evaluation of recorded tunnel evidence.

No trajectory interpolation is used: a 0.1 m coverage bin requires a real,
fresh Gazebo sample in that bin.  A passed row requires fresh contact evidence,
an actually queried configuration, and a continuous *final* stopped interval.
Synthetic tests exercise these rules but do not constitute Gazebo validation.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any, Iterable


DEFAULT_CASES = Path(__file__).with_name("cases.json")
WHEEL_COLLISIONS = tuple(
    f"{prefix}_wheel::tire"
    for prefix in ("front_left", "front_right", "rear_left", "rear_right")
)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value: Any, size: int) -> bool:
    return isinstance(value, (list, tuple)) and len(value) == size and all(map(_finite, value))


def _norm(vector: Iterable[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def load_cases(path: str | Path | None = None) -> dict:
    """Load and validate the repository-owned cases, without importing ROS."""
    config = json.loads(Path(path or DEFAULT_CASES).read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported tunnel case schema")
    offsets = config.get("offsets_m")
    if not isinstance(offsets, list) or not offsets or not all(map(_finite, offsets)):
        raise ValueError("offsets_m must be a nonempty finite list")
    if len(set(offsets)) != len(offsets):
        raise ValueError("Duplicate offsets are not independent test cases")
    defaults = config.get("defaults", {})
    if not defaults or not all(_finite(v) and v > 0 for v in defaults.values()):
        raise ValueError("Every metric threshold must be finite and positive")
    if defaults["minimum_core_coverage"] > 1:
        raise ValueError("Coverage must lie in (0, 1]")
    names = set()
    for case in config.get("tunnels", []):
        name = case.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise ValueError("Each tunnel must have a unique name")
        names.add(name)
        for field in ("start", "goal", "core_start", "core_end"):
            if not _vector(case.get(field), 2):
                raise ValueError(f"{name}: {field} must be a finite XY pair")
        for field in ("initial_z", "spawn_z", "core_half_width_m"):
            if not _finite(case.get(field)) or case[field] <= 0:
                raise ValueError(f"{name}: invalid {field}")
        direction = [b - a for a, b in zip(case["core_start"], case["core_end"])]
        length = _norm(direction)
        if length <= 0 or math.dist(case["start"], case["goal"]) <= 0:
            raise ValueError(f"{name}: degenerate centerline")
        direction = [v / length for v in direction]
        along = lambda p: sum((p[i] - case["core_start"][i]) * direction[i] for i in range(2))
        if along(case["start"]) > -defaults["end_margin_m"] or along(case["goal"]) < length + defaults["end_margin_m"]:
            raise ValueError(f"{name}: start/goal must bracket both core ends")
    if not names:
        raise ValueError("No tunnel cases")
    return config


def expand_cases(config: dict | None = None) -> list[dict]:
    """Produce all cases; positive offset is +X for blue and +Y for red."""
    config = config or load_cases()
    result = []
    for base in config["tunnels"]:
        dx, dy = [b - a for a, b in zip(base["core_start"], base["core_end"])]
        length = math.hypot(dx, dy)
        normal = [-dy / length, dx / length]
        for index, offset in enumerate(config["offsets_m"], 1):
            case = copy.deepcopy(base)
            case.update(tunnel=case["name"], trial=index,
                        case_id=f"{case['name']}-{index}", start_offset_m=offset,
                        center_start=list(base["start"]), criteria=dict(config["defaults"]))
            case["start"] = [value + offset * axis for value, axis in zip(base["start"], normal)]
            result.append(case)
    return result


def chassis_tilt_deg(quaternion: Iterable[float]) -> float:
    """Angle between chassis +Z and world +Z, including combined roll/pitch."""
    if not _vector(quaternion, 4):
        raise ValueError("Invalid chassis quaternion")
    norm = _norm(quaternion)
    if abs(norm - 1.0) > 0.05:
        raise ValueError("Chassis quaternion is not normalized")
    x, y, _, _ = [value / norm for value in quaternion]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x*x + y*y)))))


def classify_contact(a: str, b: str, normal: Iterable[float], depth_m: float,
                     robot_name: str = "red_standard_robot1") -> str:
    """Allow only a known tire against the environment with a support normal.

    Gazebo scoped names identify both collision participants.  Unknown names,
    self-contact, body contact, invalid normals/depths, and predominantly
    horizontal normals fail closed.  This is a declared new test policy, not
    a claim that the old private acceptance script checked these contacts.
    """
    if not isinstance(a, str) or not isinstance(b, str):
        return "forbidden"
    if not _vector(normal, 3) or not _finite(depth_m) or depth_m < -1e-6:
        return "forbidden"
    normal_length = _norm(normal)
    if normal_length <= 1e-9:
        return "forbidden"
    marker = robot_name + "::"
    is_robot = lambda name: name.startswith(marker) or ("::" + marker) in name
    a_robot, b_robot = is_robot(a), is_robot(b)
    if a_robot == b_robot:  # Neither (unverifiable), or self-collision.
        return "forbidden"
    wheel = a if a_robot else b
    if not any(wheel.endswith("::" + suffix) for suffix in WHEEL_COLLISIONS):
        return "forbidden"
    return "support" if abs(normal[2]) / normal_length >= 0.5 else "forbidden"


def _fresh(data: Any, sim: float, wall: float, sim_limit: float,
           wall_limit: float, stamp_key: str = "stamp") -> bool:
    if not isinstance(data, dict):
        return False
    stamp, received = data.get(stamp_key), data.get("received_wall")
    return (_finite(stamp) and _finite(received)
            and -0.05 <= sim - stamp <= sim_limit + 1e-9
            and -0.001 <= wall - received <= wall_limit + 1e-9)


def _truth_valid(data: Any) -> bool:
    if not isinstance(data, dict) or not all(_finite(data.get(k)) for k in ("x", "y", "z")):
        return False
    if not _vector(data.get("linear"), 3) or not _vector(data.get("angular"), 3):
        return False
    try:
        chassis_tilt_deg(data.get("q"))
    except (ValueError, TypeError):
        return False
    return True


def evaluate_trial(case: dict, records: Iterable[dict], metadata: dict | None = None) -> dict:
    """Evaluate one traversal from canonical sample/event JSON objects.

    Events: goal, task(state), interruption(reason), execution, recovery(status),
    and contact(contacts). Samples contain sim_time, wall_time, truth, command,
    odom, and contacts. See tests for an executable minimal schema example.
    Metadata must contain runtime_verified=True, smoother_enabled=False, and
    map_sha256_ok=True; recorded positions alone can never pass a trial.
    """
    metadata = metadata or {}
    config = load_cases()
    criteria = {**config["defaults"], **case.get("criteria", {})}
    failures: set[str] = set()
    core_start, core_end = case["core_start"], case["core_end"]
    length = math.dist(core_start, core_end)
    direction = [(b - a) / length for a, b in zip(core_start, core_end)]
    normal = [-direction[1], direction[0]]
    bin_count = math.ceil(length / criteria["coverage_bin_m"] - 1e-9)
    bins: set[int] = set()
    active = False
    goal_id = None
    task_state = None
    interruptions: list[dict] = []
    executions: list[dict] = []
    recoveries: list[dict] = []
    odom_gaps: list[dict] = []
    open_gap = None
    contacts_seen: set[tuple] = set()
    forbidden_contacts: list[dict] = []
    support_contact_count = 0
    before_time = after_time = None
    stopped_since = None
    stopped_duration = 0.0
    max_stopped_duration = 0.0
    max_tilt = max_cross = max_localization_error = None
    first_truth = final_truth = None
    previous_sample = previous_truth = None
    previous_record_wall = None
    previous_sim = None
    final_sample_valid = False
    sample_count = 0

    def consume_contacts(payload: Any, sim: float, wall: float) -> None:
        nonlocal support_contact_count
        if not isinstance(payload, dict) or payload.get("checked") is not True:
            failures.add("contact_evidence_missing")
            return
        entries = payload.get("collisions")
        if not isinstance(entries, list):
            failures.add("contact_evidence_invalid")
            return
        for contact in entries:
            if not isinstance(contact, dict):
                failures.add("contact_evidence_invalid")
                continue
            a, b = contact.get("a"), contact.get("b")
            if not isinstance(a, str) or not isinstance(b, str):
                failures.add("contact_evidence_invalid")
                continue
            # Raw geometry is mandatory. Never trust a 'support' label without
            # inspectable collision names, normal, and finite depth.
            kind = classify_contact(a, b, contact.get("normal"), contact.get("depth_m"))
            if kind not in ("support", "forbidden"):
                failures.add("contact_evidence_invalid")
                continue
            key = (payload.get("stamp"), a, b, kind,
                   repr(contact.get("normal")), repr(contact.get("depth_m")))
            if key in contacts_seen:
                continue
            contacts_seen.add(key)
            if kind == "support":
                support_contact_count += 1
            else:
                forbidden_contacts.append({"sim_time": sim, "wall_time": wall, **contact, "kind": kind})

    for record in records:
        if not isinstance(record, dict):
            failures.add("invalid_record")
            continue
        sim, wall = record.get("sim_time"), record.get("wall_time")
        if not _finite(sim) or not _finite(wall) or sim < 0:
            failures.add("invalid_record_time")
            continue
        if previous_record_wall is not None and wall < previous_record_wall - 1e-6:
            failures.add("nonmonotonic_wall_time")
        if previous_sim is not None and sim < previous_sim - 0.05:
            failures.add("simulation_clock_regressed")
        previous_record_wall, previous_sim = wall, sim
        kind = record.get("type")
        if kind in ("error", "timeout", "aborted"):
            failures.add("collector_" + kind)
            continue
        if kind == "goal":
            if active:
                failures.add("multiple_goals_in_one_trial")
            active = True
            goal_id = record.get("goal_id")
            task_state = None
            continue
        if not active:
            continue
        if kind == "task":
            if goal_id is None or record.get("goal_id", goal_id) == goal_id:
                task_state = record.get("state")
            continue
        if kind == "interruption":
            interruptions.append({"sim_time": sim, "wall_time": wall,
                                  "reason": record.get("reason", "unspecified"),
                                  "automatically_recovered": False})
            continue
        if kind == "execution":
            executions.append({"sim_time": sim, "wall_time": wall})
            continue
        if kind == "recovery":
            recoveries.append({"sim_time": sim, "wall_time": wall,
                               "status": record.get("status", "unspecified")})
            continue
        if kind == "contact":
            consume_contacts(record.get("contacts"), sim, wall)
            continue
        if kind != "sample":
            continue
        sample_count += 1
        truth, command, odom, contacts = [record.get(key) for key in ("truth", "command", "odom", "contacts")]
        truth_ok = (_truth_valid(truth) and _fresh(
            truth, sim, wall, criteria["truth_max_sim_age_s"], criteria["truth_max_wall_age_s"]))
        command_ok = (isinstance(command, dict) and _vector(command.get("linear"), 3)
                      and _vector(command.get("angular"), 3) and _fresh(
                          command, sim, wall, criteria["command_max_sim_age_s"],
                          criteria["command_max_wall_age_s"], "received_sim"))
        contacts_ok = (_fresh(contacts, sim, wall, criteria["contacts_max_sim_age_s"],
                             criteria["contacts_max_wall_age_s"])
                       and isinstance(contacts, dict) and contacts.get("checked") is True
                       and bool(contacts.get("collisions")))
        if not truth_ok:
            failures.add("truth_missing_invalid_or_stale")
        if not command_ok:
            failures.add("command_missing_invalid_or_stale")
        if not contacts_ok:
            failures.add("contact_evidence_missing_or_stale")
        else:
            consume_contacts(contacts, sim, wall)
        consecutive = True
        if previous_sample is not None:
            dt = sim - previous_sample["sim_time"]
            dw = wall - previous_sample["wall_time"]
            consecutive = (0 < dt <= criteria["maximum_sample_gap_s"] + 1e-9
                           and 0 <= dw <= criteria["maximum_sample_wall_gap_s"] + 1e-9)
            if dt > criteria["maximum_sample_gap_s"] + 1e-9 or dw > criteria["maximum_sample_wall_gap_s"] + 1e-9:
                failures.add("sample_gap_exceeds_limit")
            if dt <= 0:
                stopped_since = None
        previous_sample = record
        final_sample_valid = truth_ok and command_ok and contacts_ok
        odom_ok = (_fresh(odom, sim, wall, criteria["odom_timeout_s"], criteria["odom_timeout_s"])
                   and isinstance(odom, dict) and _finite(odom.get("x")) and _finite(odom.get("y")))
        if not odom_ok and open_gap is None:
            open_gap = {"start_sim_time": sim, "start_wall_time": wall,
                        "end_sim_time": None, "end_wall_time": None,
                        "automatically_recovered": False}
            odom_gaps.append(open_gap)
        elif odom_ok and open_gap is not None:
            open_gap.update(end_sim_time=sim, end_wall_time=wall,
                            duration_sim_s=sim-open_gap["start_sim_time"],
                            duration_wall_s=wall-open_gap["start_wall_time"])
            open_gap = None
        if truth_ok:
            position = [truth["x"], truth["y"]]
            if first_truth is None:
                first_truth = list(position)
            final_truth = list(position)
            tilt = chassis_tilt_deg(truth["q"])
            max_tilt = max(tilt, max_tilt or 0.0)
            along = sum((position[i]-core_start[i])*direction[i] for i in range(2))
            cross = sum((position[i]-core_start[i])*normal[i] for i in range(2))
            physical_position = [truth["x"], truth["y"], truth["z"]]
            if previous_truth is not None:
                previous_stamp, previous_position = previous_truth
                stamp_delta = truth["stamp"] - previous_stamp
                step = math.dist(physical_position, previous_position)
                if stamp_delta < -1e-6 or step > max(0.0, stamp_delta) * criteria["maximum_truth_speed_mps"] + criteria["maximum_truth_step_slack_m"]:
                    failures.add("truth_discontinuity_or_teleport")
            previous_truth = (truth["stamp"], physical_position)
            if 0 <= along <= length + 1e-9:
                max_cross = max(abs(cross), max_cross or 0.0)
                if abs(cross) <= case["core_half_width_m"] + 1e-9:
                    bins.add(min(bin_count-1, int(max(0.0, along) / criteria["coverage_bin_m"])))
            if abs(cross) <= criteria["end_half_width_m"] + 1e-9:
                if along <= -criteria["end_margin_m"] + 1e-9 and before_time is None:
                    before_time = sim
                if along >= length + criteria["end_margin_m"] - 1e-9 and before_time is not None and sim > before_time:
                    after_time = sim
            if odom_ok:
                localization_error = math.dist(position, [odom["x"], odom["y"]])
                max_localization_error = max(localization_error, max_localization_error or 0.0)
        moving = (truth_ok and command_ok and odom_ok
                  and _norm(truth["linear"]) >= criteria["stopped_speed_mps"]
                  and _norm(command["linear"]) >= criteria["stopped_speed_mps"])
        if moving:
            for event in interruptions:
                if (not event["automatically_recovered"] and sim > event["sim_time"]
                        and odom["stamp"] > event["sim_time"]
                        and odom["received_wall"] >= event["wall_time"]):
                    event.update(automatically_recovered=True, resumed_sim_time=sim,
                                 resumed_wall_time=wall,
                                 duration_sim_s=sim-event["sim_time"],
                                 duration_wall_s=wall-event["wall_time"])
            for gap in odom_gaps:
                if gap["end_sim_time"] is not None and not gap["automatically_recovered"]:
                    gap.update(automatically_recovered=True, motion_resumed_sim_time=sim,
                               motion_resumed_wall_time=wall)
        stopped = (final_sample_valid and task_state == "SUCCEEDED"
                   and math.dist(final_truth, case["goal"]) < criteria["maximum_goal_error_m"]
                   and _norm(truth["linear"]) < criteria["stopped_speed_mps"]
                   and _norm(truth["angular"]) < criteria["stopped_angular_speed_rps"]
                   and _norm(command["linear"]) < criteria["stopped_speed_mps"]
                   and _norm(command["angular"]) < criteria["stopped_angular_speed_rps"])
        if stopped:
            if stopped_since is None or not consecutive:
                stopped_since = sim
            stopped_duration = sim - stopped_since
            max_stopped_duration = max(stopped_duration, max_stopped_duration)
        else:
            stopped_since = None
            stopped_duration = 0.0

    if not active:
        failures.add("goal_not_recorded")
    if sample_count < 2:
        failures.add("insufficient_samples")
    final_error = math.dist(final_truth, case["goal"]) if final_truth is not None else None
    start_error = math.dist(first_truth, case["start"]) if first_truth is not None else None
    coverage = len(bins) / bin_count
    if coverage + 1e-9 < criteria["minimum_core_coverage"]:
        failures.add("insufficient_core_coverage")
    if before_time is None or after_time is None:
        failures.add("both_ends_not_crossed_in_order")
    if start_error is None or start_error > criteria["maximum_start_error_m"]:
        failures.add("requested_start_not_observed")
    if final_error is None or final_error >= criteria["maximum_goal_error_m"]:
        failures.add("final_goal_error")
    if not final_sample_valid:
        failures.add("final_evidence_not_fresh")
    if task_state != "SUCCEEDED":
        failures.add("task_not_succeeded")
    if stopped_duration + 1e-9 < criteria["minimum_stopped_duration_s"]:
        failures.add("final_continuous_stop_too_short")
    if max_tilt is None or max_tilt >= criteria["maximum_tilt_deg"]:
        failures.add("maximum_tilt_exceeded_or_missing")
    if forbidden_contacts:
        failures.add("forbidden_collision")
    if support_contact_count == 0:
        failures.add("no_physical_contact_heartbeat")
    if metadata.get("runtime_verified") is not True:
        failures.add("runtime_configuration_not_verified")
    if metadata.get("smoother_enabled") is not False:
        failures.add("smoother_not_confirmed_disabled")
    if metadata.get("map_sha256_ok") is not True:
        failures.add("original_map_hash_not_verified")
    if metadata.get("raw_path_output_identical") is False:
        failures.add("raw_path_output_changed")
    if open_gap is not None and previous_sample is not None:
        open_gap.update(duration_sim_s=previous_sample["sim_time"]-open_gap["start_sim_time"],
                        duration_wall_s=previous_sample["wall_time"]-open_gap["start_wall_time"])
    result = {
        "passed": not failures,
        "case_id": case.get("case_id", f"{case['name']}-{case.get('trial', 1)}"),
        "tunnel": case.get("tunnel", case["name"]),
        "trial": case.get("trial", 1),
        "start_offset_m": case.get("start_offset_m", 0.0),
        "final_error_m": final_error,
        "start_error_m": start_error,
        "core_coverage": coverage,
        "core_bins_observed": len(bins),
        "core_bins_total": bin_count,
        "missing_core_bins": sorted(set(range(bin_count))-bins),
        "both_ends_crossed": before_time is not None and after_time is not None,
        "stopped_duration_s": stopped_duration,
        "maximum_stopped_interval_s": max_stopped_duration,
        "task_succeeded": task_state == "SUCCEEDED",
        "smoother_enabled": metadata.get("smoother_enabled"),
        "map_yaml": metadata.get("map_yaml"),
        "controller_collision_margin_m": metadata.get("controller_collision_margin_m"),
        "max_chassis_tilt_deg": max_tilt,
        "max_cross_track_in_core_m": max_cross,
        "max_localization_error_m": max_localization_error,
        "execution_interruptions": len(interruptions),
        "automatically_recovered_interruptions": sum(event["automatically_recovered"] for event in interruptions),
        "interruption_events": interruptions,
        "odometry_gaps": odom_gaps,
        "path_executions": len(executions),
        "recovery_reverses": sum(event["status"] == "STUCK_REVERSE" for event in recoveries),
        "recovery_events": recoveries,
        "raw_path_output_identical": metadata.get("raw_path_output_identical"),
        "forbidden_contact_count": len(forbidden_contacts),
        "forbidden_contacts": forbidden_contacts,
        "support_contact_count": support_contact_count,
        "sample_count": sample_count,
        "failure_reasons": sorted(failures),
        "criteria": criteria,
    }
    return result


def aggregate_results(rows: Iterable[dict], expected_cases: Iterable[dict],
                      execution_backend: str) -> dict:
    """Aggregate an exact manifest; a partial or duplicated suite cannot pass."""
    rows, expected_cases = list(rows), list(expected_cases)
    expected_ids = [case.get("case_id", f"{case['name']}-{case.get('trial', 1)}") for case in expected_cases]
    observed_ids = [row.get("case_id") for row in rows]
    duplicates = sorted({case_id for case_id in observed_ids if observed_ids.count(case_id) > 1}, key=str)
    missing = sorted(set(expected_ids)-set(observed_ids))
    unexpected = sorted(set(observed_ids)-set(expected_ids), key=str)
    exact = (bool(expected_ids) and not duplicates and not missing and not unexpected
             and len(expected_ids) == len(set(expected_ids)))
    return {
        "schema_version": 1,
        "execution_backend": execution_backend,
        "is_gazebo_measurement": execution_backend == "gazebo",
        "expected_trials": len(expected_ids),
        "completed_trials": len(rows),
        "passed_trials": sum(row.get("passed") is True for row in rows),
        "all_passed": exact and all(row.get("passed") is True for row in rows),
        "missing_cases": missing,
        "duplicate_cases": duplicates,
        "unexpected_cases": unexpected,
        "rows": rows,
    }
