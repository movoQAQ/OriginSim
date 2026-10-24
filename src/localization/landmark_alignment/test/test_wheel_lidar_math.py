import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wheel_lidar_math import (
    body_attitude, fresh, rolling_velocity, wall_correction,
    match_fixed_planes, scan_velocity,
)


def test_sensor_mount_and_ground_velocity_past_pitch_singularity():
    mount = Rotation.from_euler('z', -1.095)
    for pitch in [0, 80, 90, 100, 170]:
        body = Rotation.from_euler('z', 20, degrees=True) * Rotation.from_euler('y', pitch, degrees=True)
        measured = mount.inv() * body * mount
        actual = body_attitude(mount, measured, Rotation.identity())
        np.testing.assert_allclose(actual.as_matrix(), body.as_matrix(), atol=1e-12)
        heading = Rotation.from_euler('z', 20, degrees=True).apply([1, 0, 0])
        np.testing.assert_allclose(rolling_velocity(actual, [(0, 1)] * 4), heading, atol=1e-12)


def test_vertical_axles_reject_unobservable_ground_motion():
    assert rolling_velocity(Rotation.from_euler('x', 90, degrees=True), [(0, 1)] * 4) is None


def test_frozen_and_future_inputs_are_rejected():
    assert fresh(10, 20, 10.05, 20.1)
    assert not fresh(10, 20, 10.11, 20.1)
    assert not fresh(10, 20, 10.05, 20.31)
    assert not fresh(10.03, 20, 10, 20.1)


def test_two_axis_wall_support_and_encoder_slip_correction():
    tangent = np.linspace(1, 8, 160)
    world = np.vstack([np.c_[np.zeros(160), tangent], np.c_[tangent, np.zeros(160)]])
    world += [0.12, -0.08]
    np.testing.assert_allclose(wall_correction(world, [(0, 0), (1, 0)]), [-0.12, 0.08], atol=1e-12)
    assert wall_correction(world[:160], [(0, 0), (1, 0)]) is None
    # A remote, unsupported plane must not repair a lost pose.
    assert wall_correction(world + [2, 2], [(0, 0), (1, 0)]) is None


def test_reacquisition_needs_independent_consistent_wall_constraints():
    fixed = [(0, 0), (0, 29), (1, 0), (1, 16)]
    np.testing.assert_allclose(match_fixed_planes([(0, .65), (0, 29.65), (1, -.08)], fixed), [-.65, .08])
    assert match_fixed_planes([(0, .65), (1, -.08)], fixed) is None
    assert match_fixed_planes([(0, .65), (0, 29.4), (1, -.08)], fixed) is None
    assert match_fixed_planes([(0, 2), (0, 31), (1, 0)], fixed) is None


def test_scan_velocity_separates_wheel_slip_from_actual_motion():
    times = [.0, .1, .2, .3]
    stopped = [(t, np.array([4., 2.])) for t in times]
    np.testing.assert_allclose(scan_velocity(stopped), [0, 0])
    moving = [(t, np.array([4.+.7*t, 2.-.2*t])) for t in times]
    np.testing.assert_allclose(scan_velocity(moving), [.7, -.2], atol=1e-12)
    assert scan_velocity(moving[:2]) is None
