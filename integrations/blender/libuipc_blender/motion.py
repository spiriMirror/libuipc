# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Serialize controller animation and sample rigid targets at solver substeps."""

import hashlib
import json
import math

import bpy
import numpy as np

from .protocol import motion_hash as sample_hash


TRANSFORMS = (
    "location",
    "rotation_euler",
    "rotation_quaternion",
    "rotation_axis_angle",
    "scale",
)


def controller_signature(target):
    """Hash authored controls without including frame-dependent channel values.

    This makes cache checks cheap and independent of the playback frame. Target
    hierarchies support keyframed object transforms; NLA, drivers, constraints,
    bone parenting and F-curve modifiers need separate evaluation contracts.
    """
    hierarchy = []
    seen = set()
    while target is not None:
        if target.name in seen:
            raise ValueError("Cyclic robot controller hierarchy")
        seen.add(target.name)
        if (
            target.library
            or target.constraints
            or target.rigid_body
            or target.parent_type != "OBJECT"
        ):
            raise ValueError(
                f"{target.name}: use local object-parented transform controllers"
            )
        state = {key: list(getattr(target, key)) for key in TRANSFORMS}
        state.update(
            name=target.name,
            rotation_mode=target.rotation_mode,
            parent_inverse=[list(row) for row in target.matrix_parent_inverse],
            delta_location=list(target.delta_location),
            delta_rotation_euler=list(target.delta_rotation_euler),
            delta_rotation_quaternion=list(target.delta_rotation_quaternion),
            delta_scale=list(target.delta_scale),
        )
        animation = target.animation_data
        curves = []
        if animation:
            if animation.drivers or animation.nla_tracks:
                raise ValueError(
                    f"{target.name}: robot targets do not support drivers or NLA"
                )
            if animation.action:
                for curve in animation.action.fcurves:
                    if curve.data_path not in TRANSFORMS or curve.modifiers:
                        raise ValueError(
                            f"{target.name}: unsupported animated target channel/modifier: {curve.data_path}"
                        )
                    keyframes = [
                        {
                            "co": list(key.co),
                            "left": list(key.handle_left),
                            "right": list(key.handle_right),
                            "interpolation": key.interpolation,
                            "easing": key.easing,
                            "amplitude": key.amplitude,
                            "back": key.back,
                            "period": key.period,
                        }
                        for key in curve.keyframe_points
                    ]
                    curves.append(
                        {
                            "path": curve.data_path,
                            "index": curve.array_index,
                            "mute": curve.mute,
                            "extrapolation": curve.extrapolation,
                            "keys": keyframes,
                        }
                    )
                    if not curve.mute and keyframes:
                        state[curve.data_path][curve.array_index] = None
        state["curves"] = curves
        hierarchy.append(state)
        target = target.parent
    return hashlib.sha256(
        json.dumps(hierarchy, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def drive_material(obj):
    body = obj.uipc_body
    if not body.driven:
        return None
    if body.role != "RIGID" or body.fixed:
        raise ValueError(f"{obj.name}: drive targets require an unfixed ABD body")
    target = body.drive_target
    if target is None or target == obj:
        raise ValueError(f"{obj.name}: assign a separate motion controller")
    return {
        "target": target.name,
        "signature": controller_signature(target),
        "translation_strength": body.drive_translation_strength,
        "rotation_strength": body.drive_rotation_strength,
        "group": body.drive_group,
        "friction": body.drive_friction,
    }


def sample_targets(scene, bodies):
    targets = {
        body["material"]["drive"]["target"]
        for body in bodies
        if "drive" in body["material"]
    }
    if not targets:
        return {}
    for name in targets:
        if name not in scene.objects:
            raise ValueError(
                f"Motion target is not linked to the simulation scene: {name}"
            )
    saved_frame, saved_subframe = scene.frame_current, scene.frame_subframe
    count = (scene.frame_end - scene.frame_start) * scene.uipc_settings.substeps + 1
    samples = {name: np.empty((count, 4, 4), dtype=np.float64) for name in targets}
    try:
        for step in range(count):
            time = scene.frame_start + step / scene.uipc_settings.substeps
            frame = math.floor(time)
            scene.frame_set(frame, subframe=time - frame)
            deps = scene.view_layers[0].depsgraph
            deps.update()
            for name in targets:
                controller = scene.objects[name]
                limits = controller.get("uipc_joint_limits")
                if (
                    limits is not None
                    and not limits[0] - 1e-6
                    <= controller.rotation_axis_angle[0]
                    <= limits[1] + 1e-6
                ):
                    raise ValueError(
                        f"{name}: joint angle exceeds URDF limits at frame {time}"
                    )
                samples[name][step] = np.array(
                    scene.objects[name].evaluated_get(deps).matrix_world
                )
    finally:
        scene.frame_set(saved_frame, subframe=saved_subframe)
    for name, matrices in samples.items():
        if not np.isfinite(matrices).all() or np.linalg.cond(matrices[0][:3, :3]) > 1e8:
            raise ValueError(f"{name}: invalid target transforms")
        relative = matrices @ np.linalg.inv(matrices[0])
        rotations = relative[:, :3, :3]
        if not np.allclose(
            rotations @ np.transpose(rotations, (0, 2, 1)), np.eye(3), atol=2e-5
        ) or not np.allclose(np.linalg.det(rotations), 1, atol=2e-5):
            raise ValueError(
                f"{name}: target motion changes scale/shear; use rigid motion"
            )
    return samples


def align_robot_initial(scene):
    bodies = [
        obj
        for obj in scene.objects
        if obj.get("uipc_robot_link") and obj.uipc_body.driven
    ]
    if not bodies:
        return
    saved_frame, saved_subframe = scene.frame_current, scene.frame_subframe
    try:
        scene.frame_set(scene.frame_start)
        deps = scene.view_layers[0].depsgraph
        deps.update()
        poses = []
        for obj in bodies:
            target = obj.uipc_body.drive_target
            if target is None:
                raise ValueError(f"{obj.name}: missing robot controller")
            poses.append((obj, target.evaluated_get(deps).matrix_world.copy()))
        for obj, pose in poses:
            obj.matrix_world = pose
    finally:
        scene.frame_set(saved_frame, subframe=saved_subframe)
