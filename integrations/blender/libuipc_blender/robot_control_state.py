# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Portable pose compatibility and sampled joint-angle diagnostics."""

import hashlib
import json
import math

import numpy as np


def layout_signature(joints):
    fields = sorted(({k: joint[k] for k in ("name", "kind", "limits", "axis")} for joint in joints),
                    key=lambda joint: joint["name"])
    return hashlib.sha256(json.dumps(fields, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_pose(pose, joints):
    movable = {j["name"]: j for j in joints if j["kind"] == "revolute"}
    if (not isinstance(pose, dict) or pose.get("schema_version") != 1
            or pose.get("layout") != layout_signature(joints)
            or not isinstance(pose.get("angles"), dict) or set(pose["angles"]) != set(movable)):
        raise ValueError("Pose belongs to a different joint layout or has missing joints")
    angles = {}
    for name, value in pose["angles"].items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"{name}: pose angle must be finite")
        limits = movable[name]["limits"]
        if limits and not limits[0] - 1e-6 <= value <= limits[1] + 1e-6:
            raise ValueError(f"{name}: pose exceeds the URDF limits")
        angles[name] = min(limits[1], max(limits[0], value)) if limits else value
    return angles


def trajectory_metrics(values, fps, substeps, first, limits=None):
    angles = np.asarray(values, dtype=float)
    if angles.ndim != 1 or not len(angles) or not np.isfinite(angles).all() or fps <= 0 or substeps < 1:
        raise ValueError("Invalid joint trajectory samples")
    velocity = np.diff(angles) * fps * substeps
    acceleration = np.diff(velocity) * fps * substeps
    speed_index = int(np.argmax(np.abs(velocity))) if len(velocity) else 0
    acceleration_index = int(np.argmax(np.abs(acceleration))) if len(acceleration) else 0
    violations = np.flatnonzero((angles < limits[0] - 1e-6) | (angles > limits[1] + 1e-6)) if limits else []
    return {"max_speed_deg_s": math.degrees(float(np.max(np.abs(velocity)))) if len(velocity) else 0,
            "max_acceleration_deg_s2": math.degrees(float(np.max(np.abs(acceleration)))) if len(acceleration) else 0,
            "speed_frame": first + (speed_index + 1) / substeps if len(velocity) else first,
            "acceleration_frame": first + (acceleration_index + 2) / substeps if len(acceleration) else first,
            "limit_frame": first + int(violations[0]) / substeps if len(violations) else None}
