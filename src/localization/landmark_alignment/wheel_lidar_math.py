"""Geometry and validity checks for the calibrated planar odometry backend."""
import math
import numpy as np


def body_attitude(mount, measured, reference):
    return mount * measured * reference.inv() * mount.inv()


def rolling_velocity(rotation, wheels):
    velocity = np.zeros(3)
    for steering, speed in wheels:
        axle = rotation.apply([-math.sin(steering), math.cos(steering), 0])
        length = math.hypot(axle[0], axle[1])
        if length < 0.1:
            return None
        velocity[:2] += speed * np.array([axle[1], -axle[0]]) / (len(wheels) * length)
    return velocity


def fresh(stamp, receipt, now, wall):
    return -0.02 <= now - stamp <= 0.1 and 0 <= wall - receipt <= 0.3


def wall_correction(world, planes):
    residual = np.stack([np.abs(world[:, axis] - d) for axis, d in planes], axis=1)
    labels = residual.argmin(1)
    keep = residual.min(1) < 0.3
    correction = np.zeros(2)
    for axis in (0, 1):
        errors = []
        for label, (normal, distance) in enumerate(planes):
            if normal != axis:
                continue
            points = world[keep & (labels == label)]
            if len(points) < 30 or np.quantile(points[:, 1-axis], 0.95) - np.quantile(points[:, 1-axis], 0.05) < 2:
                continue
            errors.extend((distance - points[:, axis]).tolist())
        if len(errors) < 40:
            return None
        correction[axis] = np.median(errors)
    if np.max(np.abs(correction)) > 0.3:
        return None
    # A 10 Hz scan may need to remove 0.1 m of encoder motion during slip.
    return np.clip(correction, -0.15, 0.15)


def match_fixed_planes(detected, fixed):
    """Relocalize only with three consistent walls spanning both axes."""
    if detected is None:
        return None
    matches = {}
    for axis, distance in detected:
        candidates = [(abs(distance - d), i, d) for i, (a, d) in enumerate(fixed) if a == axis]
        if not candidates:
            continue
        error, index, target = min(candidates)
        if error > 1.0 or index in matches:
            return None
        matches[index] = (axis, target - distance)
    if len(matches) < 3:
        return None
    correction = []
    for axis in (0, 1):
        errors = [e for a, e in matches.values() if a == axis]
        if not errors or max(errors) - min(errors) > 0.06:
            return None
        correction.append(float(np.median(errors)))
    return np.array(correction)


def scan_velocity(history):
    if len(history) < 3 or history[-1][0] - history[0][0] < 0.15:
        return None
    stamps = np.array([t for t, _ in history])
    positions = np.array([p for _, p in history])
    centered = stamps - stamps.mean()
    velocity = centered @ (positions - positions.mean(0)) / (centered @ centered)
    return velocity if np.isfinite(velocity).all() else None
