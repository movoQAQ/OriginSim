"""Synthetic evaluator tests. These are not physical Gazebo runs."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))

from tunnel_regression.metrics import (  # noqa: E402
    aggregate_results, chassis_tilt_deg, classify_contact, evaluate_trial,
    expand_cases, load_cases,
)


METADATA = {"runtime_verified": True, "smoother_enabled": False,
            "map_sha256_ok": True, "raw_path_output_identical": True}


def support_contact():
    return {"a": "red_standard_robot1::front_left_wheel::tire",
            "b": "rmuc_2026::field::collision", "normal": [0, 0, -1],
            "depth_m": 0.0001, "kind": "support"}


def sample(t, x, y, velocity=None, command=None, z=0.2758):
    velocity = velocity or [0.0, 0.0, 0.0]
    command = command or list(velocity)
    return {
        "type": "sample", "sim_time": t, "wall_time": t,
        "truth": {"stamp": t, "received_wall": t, "x": x, "y": y, "z": z,
                  "q": [0, 0, 0, 1], "linear": velocity, "angular": [0, 0, 0]},
        "command": {"received_sim": t, "received_wall": t,
                    "linear": command, "angular": [0, 0, 0]},
        "odom": {"stamp": t, "received_wall": t, "x": x, "y": y},
        "contacts": {"stamp": t, "received_wall": t, "checked": True,
                     "collisions": [support_contact()]},
    }


def passing_records(case):
    """Straight known centerline with deliberate final hold; synthetic only."""
    start, goal = case["center_start"], case["goal"]
    length = math.dist(start, goal)
    d = [(goal[i]-start[i])/length for i in range(2)]
    n = [-d[1], d[0]]
    offset = case["start_offset_m"]
    entry = math.dist(start, case["core_start"])
    records = [{"type": "goal", "sim_time": 1.0, "wall_time": 1.0, "goal_id": 1},
               {"type": "execution", "sim_time": 1.0, "wall_time": 1.0}]
    steps = math.ceil(length / 0.025)
    for i in range(steps+1):
        s = min(i * 0.025, length)
        lateral = offset * max(0, 1-s/max(0.1, entry-0.3))
        p = [start[j]+d[j]*s+n[j]*lateral for j in range(2)]
        records.append(sample(1.05 + i*0.05, *p, velocity=[d[0]*0.5, d[1]*0.5, 0], z=case["initial_z"]))
    end = records[-1]["sim_time"]
    records.append({"type": "task", "state": "SUCCEEDED", "goal_id": 1,
                    "sim_time": end + 0.001, "wall_time": end + 0.001})
    for i in range(1, 67):
        records.append(sample(end + i*0.05, *goal, z=case["initial_z"]))
    return records


def samples(records):
    return [r for r in records if r["type"] == "sample"]


class MetricTests(unittest.TestCase):
    def setUp(self):
        self.cases = expand_cases()
        self.case = self.cases[0]
        self.records = passing_records(self.case)

    def evaluate(self, records=None, metadata=None):
        return evaluate_trial(self.case, records or self.records,
                              METADATA if metadata is None else metadata)

    def assert_rejected(self, records, reason):
        row = self.evaluate(records)
        self.assertFalse(row["passed"], row)
        self.assertIn(reason, row["failure_reasons"])

    def test_all_fifteen_synthetic_cases_and_offsets(self):
        self.assertEqual(len(self.cases), 15)
        rows = [evaluate_trial(case, passing_records(case), METADATA) for case in self.cases]
        for case, row in zip(self.cases, rows):
            self.assertTrue(row["passed"], (case["case_id"], row["failure_reasons"]))
            self.assertEqual(row["core_coverage"], 1.0)
            self.assertGreaterEqual(row["stopped_duration_s"], 3.0)
        report = aggregate_results(rows, self.cases, "synthetic")
        self.assertTrue(report["all_passed"])
        self.assertFalse(report["is_gazebo_measurement"])
        self.assertEqual(report["execution_backend"], "synthetic")

    def test_offset_sign_is_positive_x_blue_positive_y_red(self):
        self.assertAlmostEqual(self.cases[1]["start"][0], 19.215)
        self.assertAlmostEqual(self.cases[1]["start"][1], 14.725)
        self.assertAlmostEqual(self.cases[7]["start"][0], 11.7)
        self.assertAlmostEqual(self.cases[7]["start"][1], 14.015)

    def test_core_intervals_match_unchanged_original_pgm_strips(self):
        config = load_cases()
        provenance = config["geometry_provenance"]
        data = (TOOLS.parent / provenance["map"]).read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), provenance["map_sha256"])
        magic, comment, dimensions, maximum, pixels = data.split(b"\n", 4)
        self.assertEqual(magic, b"P5")
        self.assertTrue(comment.startswith(b"#"))
        self.assertEqual(maximum, b"255")
        width, height = map(int, dimensions.split())
        self.assertEqual(len(pixels), width*height)
        resolution = provenance["map_resolution_m"]
        origin = provenance["map_origin"]

        def free(point):
            x = math.floor((point[0]-origin[0])/resolution + 1e-9)
            y = math.floor((point[1]-origin[1])/resolution + 1e-9)
            return pixels[(height-1-y)*width+x] == 254

        for case in config["tunnels"]:
            length = math.dist(case["core_start"], case["core_end"])
            d = [(b-a)/length for a, b in zip(case["core_start"], case["core_end"])]
            n = [-d[1], d[0]]
            half_free_width = case["core_half_width_m"] + 0.30
            count = math.ceil(length/resolution)
            for i in range(count):
                s = min((i+0.5)*resolution, length-0.001)
                center = [case["core_start"][j]+d[j]*s for j in range(2)]
                self.assertTrue(free(center), (case["name"], center))
                for sign in (-1, 1):
                    inside = [center[j]+sign*n[j]*(half_free_width-0.001) for j in range(2)]
                    outside = [center[j]+sign*n[j]*(half_free_width+0.001) for j in range(2)]
                    self.assertTrue(free(inside), (case["name"], inside))
                    self.assertFalse(free(outside), (case["name"], outside))

    def test_missing_configuration_cannot_pass(self):
        for key in METADATA:
            if key == "raw_path_output_identical":
                continue
            metadata = dict(METADATA)
            del metadata[key]
            self.assertFalse(self.evaluate(metadata=metadata)["passed"], key)
        self.assertFalse(self.evaluate(metadata={})["passed"])

    def test_smoother_or_wrong_map_rejected(self):
        for key, value in [("smoother_enabled", True), ("map_sha256_ok", False),
                           ("raw_path_output_identical", False)]:
            self.assertFalse(self.evaluate(metadata={**METADATA, key: value})["passed"])

    def test_pose_offset_must_really_be_applied(self):
        wrong_case = self.cases[1]
        row = evaluate_trial(wrong_case, passing_records(self.cases[0]), METADATA)
        self.assertIn("requested_start_not_observed", row["failure_reasons"])

    def test_bypass_not_counted_as_core_coverage(self):
        for r in samples(self.records):
            if 11.6 <= r["truth"]["y"] <= 14:
                r["truth"]["x"] += 1.0
        self.assert_rejected(self.records, "insufficient_core_coverage")

    def test_no_interpolation_across_unobserved_core(self):
        records = [r for r in self.records if r["type"] != "sample" or not 11.6 <= r["truth"]["y"] <= 14]
        self.assert_rejected(records, "insufficient_core_coverage")

    def test_duplicate_samples_cannot_advance_stop_clock(self):
        stop = samples(self.records)[-1]
        records = self.records[:1] + [copy.deepcopy(stop) for _ in range(100)]
        records.insert(1, {"type": "task", "state": "SUCCEEDED", "sim_time": 1.0, "wall_time": 1.0})
        self.assert_rejected(records, "final_continuous_stop_too_short")

    def test_stale_truth_rejected_even_if_positions_look_good(self):
        samples(self.records)[-1]["truth"]["stamp"] -= 1.0
        self.assert_rejected(self.records, "truth_missing_invalid_or_stale")

    def test_wall_time_freshness_independent_of_sim_clock(self):
        samples(self.records)[-1]["truth"]["received_wall"] -= 3.0
        self.assert_rejected(self.records, "truth_missing_invalid_or_stale")

    def test_stale_command_rejected(self):
        samples(self.records)[-1]["command"]["received_sim"] -= 1.0
        self.assert_rejected(self.records, "command_missing_invalid_or_stale")

    def test_late_motion_invalidates_earlier_stopped_interval(self):
        final = samples(self.records)[-1]
        final["truth"]["linear"] = [0.1, 0, 0]
        row = self.evaluate()
        self.assertGreater(row["maximum_stopped_interval_s"], 3.0)
        self.assertEqual(row["stopped_duration_s"], 0.0)
        self.assertFalse(row["passed"])

    def test_late_position_drift_invalidates_stop(self):
        for r in samples(self.records)[-2:]:
            r["truth"]["x"] += 0.31
        self.assert_rejected(self.records, "final_goal_error")

    def test_full_3d_speed_required_for_stop(self):
        samples(self.records)[-1]["truth"]["linear"][2] = 0.1
        self.assert_rejected(self.records, "final_continuous_stop_too_short")

    def test_roll_rate_required_for_stop(self):
        samples(self.records)[-1]["truth"]["angular"][0] = 0.1
        self.assert_rejected(self.records, "final_continuous_stop_too_short")

    def test_command_angular_norm_required_for_stop(self):
        samples(self.records)[-1]["command"]["angular"][1] = 0.1
        self.assert_rejected(self.records, "final_continuous_stop_too_short")

    def test_success_must_belong_to_active_goal(self):
        for r in self.records:
            if r["type"] == "task":
                r["goal_id"] = 2
        self.assert_rejected(self.records, "task_not_succeeded")

    def test_old_success_before_goal_does_not_count(self):
        records = [{"type": "task", "state": "SUCCEEDED", "sim_time": 0.5, "wall_time": 0.5}]
        records += [r for r in self.records if r["type"] != "task"]
        self.assert_rejected(records, "task_not_succeeded")

    def test_later_failed_task_invalidates_success(self):
        t = self.records[-1]["sim_time"] + 0.01
        self.records.append({"type": "task", "state": "FAILED", "sim_time": t, "wall_time": t})
        self.assert_rejected(self.records, "task_not_succeeded")

    def test_missing_goal_never_passes(self):
        self.assert_rejected([r for r in self.records if r["type"] != "goal"], "goal_not_recorded")

    def test_nan_cannot_pass_or_escape_into_results(self):
        samples(self.records)[-1]["truth"]["x"] = float("nan")
        row = self.evaluate()
        self.assertFalse(row["passed"])
        json.dumps(row, allow_nan=False)

    def test_nan_velocity_rejected(self):
        samples(self.records)[-1]["truth"]["linear"][1] = float("nan")
        self.assert_rejected(self.records, "truth_missing_invalid_or_stale")

    def test_teleport_detected_even_if_coverage_is_elsewhere_complete(self):
        samples(self.records)[20]["truth"]["z"] += 3.0
        # Vertical teleport is not used for planar coverage, but the physical
        # full-pose discontinuity check must still detect it.
        self.assert_rejected(self.records, "truth_discontinuity_or_teleport")

    def test_sample_gap_fails_closed(self):
        del self.records[20:30]
        self.assert_rejected(self.records, "sample_gap_exceeds_limit")

    def test_clock_reset_rejected(self):
        self.records[30]["sim_time"] -= 2.0
        self.assert_rejected(self.records, "simulation_clock_regressed")

    def test_contacts_absent_is_not_no_collision(self):
        for r in samples(self.records):
            r["contacts"] = None
        self.assert_rejected(self.records, "contact_evidence_missing_or_stale")

    def test_empty_contacts_cannot_create_support_heartbeat(self):
        for r in samples(self.records):
            r["contacts"]["collisions"] = []
        self.assert_rejected(self.records, "no_physical_contact_heartbeat")

    def test_stale_contact_stream_rejected(self):
        samples(self.records)[-1]["contacts"]["stamp"] -= 3.0
        self.assert_rejected(self.records, "contact_evidence_missing_or_stale")

    def test_body_collision_even_transient_event_is_failure(self):
        t = self.records[20]["sim_time"] + 0.001
        c = support_contact()
        c["a"] = "red_standard_robot1::chassis::base_link_base_collision"
        c["kind"] = "support"  # Raw geometry must override a wrong label.
        event = {"type": "contact", "sim_time": t, "wall_time": t,
                 "contacts": {"stamp": t, "received_wall": t,
                              "checked": True, "collisions": [c]}}
        self.records.insert(21, event)
        self.assert_rejected(self.records, "forbidden_collision")

    def test_wheel_wall_collision_is_not_ground_support(self):
        c = support_contact()
        c["normal"] = [1, 0, 0]
        samples(self.records)[20]["contacts"]["collisions"] = [c]
        self.assert_rejected(self.records, "forbidden_collision")

    def test_missing_raw_contact_normal_fails_closed(self):
        c = support_contact()
        c["normal"] = None
        samples(self.records)[20]["contacts"]["collisions"] = [c]
        self.assert_rejected(self.records, "forbidden_collision")

    def test_unsubstantiated_support_label_fails_closed(self):
        c = support_contact()
        del c["normal"]
        del c["depth_m"]
        samples(self.records)[20]["contacts"]["collisions"] = [c]
        self.assert_rejected(self.records, "forbidden_collision")

    def test_late_collector_error_cannot_preserve_a_pass(self):
        t = self.records[-1]["sim_time"] + 0.1
        self.records.append({"type": "error", "sim_time": t, "wall_time": t,
                             "reason": "simulation clock stopped"})
        self.assert_rejected(self.records, "collector_error")

    def test_combined_tilt_and_yaw(self):
        self.assertAlmostEqual(chassis_tilt_deg([0, 0, math.sin(1), math.cos(1)]), 0)
        self.assertAlmostEqual(chassis_tilt_deg([math.sin(math.pi/12), 0, 0, math.cos(math.pi/12)]), 30)
        samples(self.records)[20]["truth"]["q"] = [math.sin(math.pi/8), 0, 0, math.cos(math.pi/8)]
        self.assert_rejected(self.records, "maximum_tilt_exceeded_or_missing")

    def test_odometry_gap_recovery_requires_real_motion(self):
        seq = samples(self.records)
        for r in seq[20:40]:
            r["odom"] = None
        t = seq[20]["sim_time"]
        idx = self.records.index(seq[20])
        self.records.insert(idx, {"type": "interruption", "reason": "Odometry receive timeout",
                                  "sim_time": t, "wall_time": t})
        row = self.evaluate()
        self.assertTrue(row["passed"], row["failure_reasons"])
        self.assertEqual(row["execution_interruptions"], 1)
        self.assertEqual(row["automatically_recovered_interruptions"], 1)
        self.assertEqual(len(row["odometry_gaps"]), 1)
        self.assertTrue(row["odometry_gaps"][0]["automatically_recovered"])
        self.assertAlmostEqual(row["odometry_gaps"][0]["duration_sim_s"], 1.0)

    def test_fresh_odom_while_stationary_is_not_automatic_motion_recovery(self):
        seq = samples(self.records)
        r = seq[-60]
        index = self.records.index(r)
        self.records.insert(index, {"type": "interruption", "reason": "timeout",
                                    "sim_time": r["sim_time"], "wall_time": r["wall_time"]})
        row = self.evaluate()
        self.assertEqual(row["automatically_recovered_interruptions"], 0)

    def test_incomplete_duplicate_or_unexpected_suite_cannot_pass(self):
        rows = [evaluate_trial(case, passing_records(case), METADATA) for case in self.cases]
        self.assertFalse(aggregate_results(rows[:-1], self.cases, "gazebo")["all_passed"])
        self.assertFalse(aggregate_results(rows[:-1]+rows[:1], self.cases, "gazebo")["all_passed"])
        self.assertFalse(aggregate_results(rows+rows[:1], self.cases, "gazebo")["all_passed"])
        self.assertFalse(aggregate_results([], [], "gazebo")["all_passed"])

    def test_metadata_does_not_mutate_historical_result_file(self):
        path = TOOLS.parent / "docs/no_smoother_vehicle_results.json"
        before = hashlib.sha256(path.read_bytes()).hexdigest()
        self.evaluate()
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_contact_classification_is_scope_and_normal_sensitive(self):
        c = support_contact()
        self.assertEqual(classify_contact(c["a"], c["b"], c["normal"], c["depth_m"]), "support")
        self.assertEqual(classify_contact(c["b"], c["a"], [0, 0, 1], 0), "support")
        self.assertEqual(classify_contact(c["a"], c["a"], [0, 0, 1], 0), "forbidden")
        self.assertEqual(classify_contact("other::tire", c["b"], [0, 0, 1], 0), "forbidden")
        self.assertEqual(classify_contact(c["a"], c["b"], [0, 0, 0], 0), "forbidden")
        self.assertEqual(classify_contact(c["a"], c["b"], [0, 0, 1], float("nan")), "forbidden")


if __name__ == "__main__":
    unittest.main()
