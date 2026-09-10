# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""URDF joint-angle views, robot-local poses, keying and trajectory review."""

import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import bpy
import numpy as np
from bpy.props import BoolProperty, CollectionProperty, FloatProperty, IntProperty, PointerProperty, StringProperty

from .robot_control_state import layout_signature, validate_pose, trajectory_metrics


def belongs_to(control, root):
    current = control
    while current:
        if current == root:
            return True
        current = current.parent
    return False


def active_root(context):
    root = context.scene.uipc_settings.active_robot
    if root and root.name in context.scene.objects:
        return root
    current = context.object
    if current and current.uipc_body.driven:
        current = current.uipc_body.drive_target
    while current:
        if current.get("uipc_robot_source"):
            return current
        current = current.parent
    roots = [o for o in context.scene.objects if o.get("uipc_robot_source")]
    return roots[0] if len(roots) == 1 else None


def refresh_controls(scene, root):
    metadata = {}
    source = Path(root.get("uipc_robot_source", ""))
    if source.is_file():
        try:
            for joint in ET.parse(source).getroot().findall("joint"):
                limit = joint.find("limit")
                metadata[joint.get("name")] = (joint.get("type"),
                    float(limit.get("velocity")) if limit is not None and limit.get("velocity") is not None else None)
        except (OSError, ET.ParseError, ValueError):
            metadata = {}
    controls = sorted((o for o in scene.objects if o.get("uipc_joint_name") and belongs_to(o, root)),
                      key=lambda o: tuple(int(p) if p.isdigit() else p for p in re.split(r"(\d+)", o["uipc_joint_name"])))
    names = [o["uipc_joint_name"] for o in controls]
    if len(set(names)) != len(names):
        raise ValueError("Duplicate URDF joint names beneath this robot root")
    root.uipc_robot.joints.clear()
    for control in controls:
        name = control["uipc_joint_name"]
        kind, velocity = metadata.get(name, (control.get("uipc_joint_type", "revolute" if control.get("uipc_joint_limits") is not None else "unknown"), None))
        if kind not in ("fixed", "revolute"):
            kind = "unknown"
        control["uipc_joint_type"] = kind
        if velocity is not None and math.isfinite(velocity) and velocity >= 0:
            control["uipc_joint_velocity_limit"] = velocity
        entry = root.uipc_robot.joints.add()
        entry.name, entry.control = name, control
    root["uipc_robot_joints"] = json.dumps({c["uipc_joint_name"]: c.name for c in controls})
    links = [o for o in scene.objects if o.get("uipc_robot_link") and o.uipc_body.drive_target
             and belongs_to(o.uipc_body.drive_target, root)]
    root["uipc_robot_links"] = json.dumps({o["uipc_robot_link"]: o.name for o in links})
    root["uipc_robot_targets"] = json.dumps({o["uipc_robot_link"]: o.uipc_body.drive_target.name for o in links})
    root.uipc_robot.review = ""


def joint_descriptors(root):
    if root.library:
        raise ValueError("Make the robot local before editing its controls")
    result = []
    for entry in root.uipc_robot.joints:
        control = entry.control
        if control is None or not belongs_to(control, root):
            raise ValueError("Joint list is stale; refresh it before editing")
        if control.library or control.rotation_mode != "AXIS_ANGLE":
            raise ValueError(f"{entry.name}: use local axis-angle URDF controls")
        kind = control.get("uipc_joint_type", "unknown")
        limits = control.get("uipc_joint_limits")
        limits = list(limits) if limits is not None else None
        if limits and (len(limits) != 2 or not all(math.isfinite(x) for x in limits) or limits[0] > limits[1]):
            raise ValueError(f"{entry.name}: invalid joint limits")
        result.append({"name": entry.name, "kind": kind, "limits": limits,
                       "axis": list(control.rotation_axis_angle[1:]),
                       "velocity": control.get("uipc_joint_velocity_limit")})
    return result


def mark_edited(scene, root):
    from . import invalidate
    root.uipc_robot.review = ""
    invalidate(scene, "Robot controls changed; bake again after authoring the trajectory")


def preview_pose(scene, root):
    graph = scene.view_layers[0].depsgraph
    graph.update()
    poses = [(obj, obj.uipc_body.drive_target.evaluated_get(graph).matrix_world.copy())
             for obj in scene.objects if obj.get("uipc_robot_link") and obj.uipc_body.driven
             and obj.uipc_body.drive_target and belongs_to(obj.uipc_body.drive_target, root)]
    for obj, pose in poses:
        obj.matrix_world = pose
    mark_edited(scene, root)


def key_angle(control, frame):
    control.keyframe_insert(data_path="rotation_axis_angle", index=0, frame=frame)
    curve = next(c for c in control.animation_data.action.fcurves if c.data_path == "rotation_axis_angle" and c.array_index == 0)
    for point in curve.keyframe_points:
        if abs(point.co.x - frame) < 1e-4:
            point.interpolation = "BEZIER"
            point.handle_left_type = point.handle_right_type = "AUTO_CLAMPED"


def get_angle(entry):
    return math.degrees(entry.control.rotation_axis_angle[0]) if entry.control else 0.0


def set_angle(entry, degrees):
    from . import runtime
    root, control = entry.id_data, entry.control
    if runtime.is_running() or control is None or not belongs_to(control, root) or control.get("uipc_joint_type") != "revolute":
        return
    value = math.radians(degrees)
    if not math.isfinite(value):
        return
    limits = control.get("uipc_joint_limits")
    if limits is not None:
        value = min(limits[1], max(limits[0], value))
    control.rotation_axis_angle[0] = value
    scene = bpy.context.scene
    if scene.tool_settings.use_keyframe_insert_auto:
        key_angle(control, scene.frame_current + scene.frame_subframe)
    if root.uipc_robot.live_preview:
        preview_pose(scene, root)
    mark_edited(scene, root)


class UIPCJointControl(bpy.types.PropertyGroup):
    control: PointerProperty(type=bpy.types.Object)
    angle: FloatProperty(name="Angle (deg)", get=get_angle, set=set_angle, precision=3, options=set())


class UIPCRobotPose(bpy.types.PropertyGroup):
    payload: StringProperty(options={"HIDDEN"})


class UIPCRobotSettings(bpy.types.PropertyGroup):
    joints: CollectionProperty(type=UIPCJointControl)
    active_joint: IntProperty(default=0, min=0)
    poses: CollectionProperty(type=UIPCRobotPose)
    pose_name: StringProperty(name="Joint Pose")
    live_preview: BoolProperty(name="Preview While Editing", default=True,
        description="Kinematic editing preview only; actual coupled simulation still requires Bake")
    speed_review: FloatProperty(name="Fallback Speed Review (deg/s)", default=180, min=0,
        description="Used only if the URDF supplies no velocity limit; zero disables")
    acceleration_review: FloatProperty(name="Acceleration Review (deg/s^2)", default=720, min=0)
    review: StringProperty(options={"HIDDEN", "SKIP_SAVE"})
    review_detail: BoolProperty(name="Show All Joint Results", default=False)


class RobotOperator:
    @classmethod
    def poll(cls, context):
        from . import runtime
        return not runtime.is_running() and active_root(context) is not None


class UIPC_OT_robot_refresh(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_refresh"
    bl_label = "Refresh Joint List"
    bl_description = "Find this robot's controls and restore available URDF joint metadata without changing angles"

    def execute(self, context):
        try:
            refresh_controls(context.scene, active_root(context))
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_robot_preview(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_preview"
    bl_label = "Preview Current Pose"
    bl_description = "Preview controller poses kinematically; bake to compute actual contact and deformation"

    def execute(self, context):
        preview_pose(context.scene, active_root(context))
        return {"FINISHED"}


class UIPC_OT_robot_pose_save(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_pose_save"
    bl_label = "Save Joint Pose"
    bl_description = "Save or replace a robot-local pose of all movable joints; root transforms are not stored"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        root = active_root(context)
        try:
            joints = joint_descriptors(root)
            if not joints or not root.uipc_robot.pose_name.strip():
                raise ValueError("Refresh joints and enter a pose name")
            angles = {e.name: float(e.control.rotation_axis_angle[0]) for e, j in zip(root.uipc_robot.joints, joints)
                      if j["kind"] == "revolute"}
            pose = {"schema_version": 1, "layout": layout_signature(joints), "angles": angles}
            validate_pose(pose, joints)
            name = root.uipc_robot.pose_name.strip()
            entry = root.uipc_robot.poses.get(name)
            if entry is None:
                entry = root.uipc_robot.poses.add()
                entry.name = name
            entry.payload = json.dumps(pose, allow_nan=False)
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_robot_pose_apply(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_pose_apply"
    bl_label = "Apply Joint Pose"
    bl_description = "Apply a validated joint pose; insert keys to commit edits on animated controls"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        root, scene = active_root(context), context.scene
        try:
            entry = root.uipc_robot.poses.get(root.uipc_robot.pose_name.strip())
            if entry is None:
                raise ValueError("Choose a saved joint pose")
            angles = validate_pose(json.loads(entry.payload), joint_descriptors(root))
            for joint in root.uipc_robot.joints:
                if joint.name in angles:
                    joint.control.rotation_axis_angle[0] = angles[joint.name]
                    if scene.tool_settings.use_keyframe_insert_auto:
                        key_angle(joint.control, scene.frame_current + scene.frame_subframe)
            if root.uipc_robot.live_preview:
                preview_pose(scene, root)
            mark_edited(scene, root)
        except (ValueError, TypeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_robot_key(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_key"
    bl_label = "Key Joint Angles"
    bl_description = "Key actual URDF control transforms with Auto Clamped Bezier handles"
    bl_options = {"REGISTER", "UNDO"}
    index: IntProperty(default=-1, min=-1)
    include_root: BoolProperty(default=False)

    def execute(self, context):
        root, scene = active_root(context), context.scene
        joints = joint_descriptors(root)
        if self.index >= len(joints):
            self.report({"ERROR"}, "Joint index is out of range")
            return {"CANCELLED"}
        frame = scene.frame_current + scene.frame_subframe
        for i, (entry, joint) in enumerate(zip(root.uipc_robot.joints, joints)):
            if joint["kind"] == "revolute" and (self.index < 0 or self.index == i):
                key_angle(entry.control, frame)
        if self.include_root:
            root.keyframe_insert(data_path="location", frame=frame)
            rotation = "rotation_quaternion" if root.rotation_mode == "QUATERNION" else "rotation_axis_angle" if root.rotation_mode == "AXIS_ANGLE" else "rotation_euler"
            root.keyframe_insert(data_path=rotation, frame=frame)
        mark_edited(scene, root)
        return {"FINISHED"}


class UIPC_OT_robot_review(RobotOperator, bpy.types.Operator):
    bl_idname = "uipc.robot_review"
    bl_label = "Review Joint Trajectory"
    bl_description = "Sample authored joint curves for position-limit and angular-rate review without moving the timeline"

    def execute(self, context):
        from .motion import controller_signature
        root, scene = active_root(context), context.scene
        try:
            joints = joint_descriptors(root)
            substeps = scene.uipc_settings.substeps
            count = (scene.frame_end - scene.frame_start) * substeps + 1
            if not 1 <= count <= 200000:
                raise ValueError("Trajectory review requires 1-200000 sampled times")
            times = scene.frame_start + np.arange(count) / substeps
            rows = []
            for entry, joint in zip(root.uipc_robot.joints, joints):
                if joint["kind"] != "revolute":
                    continue
                control = entry.control
                controller_signature(control)  # Reject drivers/NLA/modifiers like the bake path.
                action = control.animation_data.action if control.animation_data else None
                curve = next((c for c in action.fcurves if c.data_path == "rotation_axis_angle" and c.array_index == 0 and not c.mute), None) if action else None
                values = [curve.evaluate(float(t)) for t in times] if curve else np.full(count, control.rotation_axis_angle[0])
                metrics = trajectory_metrics(values, scene.render.fps / scene.render.fps_base, substeps,
                                             scene.frame_start, joint["limits"])
                limit = math.degrees(joint["velocity"]) if joint["velocity"] is not None else root.uipc_robot.speed_review
                warn_speed = metrics["max_speed_deg_s"] > limit if joint["velocity"] is not None or limit > 0 else False
                rows.append({"name": entry.name, **metrics, "speed_limit_deg_s": limit,
                             "warning": warn_speed or (root.uipc_robot.acceleration_review > 0 and metrics["max_acceleration_deg_s2"] > root.uipc_robot.acceleration_review) or metrics["limit_frame"] is not None})
            root.uipc_robot.review = json.dumps({"frame_start": scene.frame_start, "frame_end": scene.frame_end,
                "fps": scene.render.fps / scene.render.fps_base, "substeps": substeps, "rows": rows}, allow_nan=False)
        except (ValueError, TypeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_UL_joint_controls(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        control = item.control
        kind = control.get("uipc_joint_type", "unknown") if control else "unknown"
        row = layout.row(align=True)
        row.label(text=item.name)
        controls = row.row(align=True)
        controls.enabled = bool(control and kind == "revolute" and control.rotation_mode == "AXIS_ANGLE"
                                and belongs_to(control, item.id_data))
        controls.prop(item, "angle", text="deg")
        controls.operator("uipc.robot_key", text="Key").index = index


class UIPC_PT_robot_controls(bpy.types.Panel):
    bl_label = "Robot Joint Controls"
    bl_idname = "UIPC_PT_robot_controls"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "libuipc"

    def draw(self, context):
        from . import runtime
        layout = self.layout
        layout.ui_units_x = 28
        layout.enabled = not runtime.is_running()
        layout.prop(context.scene.uipc_settings, "active_robot")
        root = active_root(context)
        if root is None:
            layout.label(text="Import/select a URDF robot")
            return
        settings = root.uipc_robot
        layout.label(text=root.name)
        layout.operator("uipc.robot_refresh")
        layout.prop(settings, "live_preview")
        layout.prop(context.scene.tool_settings, "use_keyframe_insert_auto", text="Auto Key Joint Edits")
        layout.label(text="Preview is kinematic; Bake computes physical contact")
        layout.template_list("UIPC_UL_joint_controls", "", settings, "joints", settings, "active_joint", rows=8)
        if 0 <= settings.active_joint < len(settings.joints):
            entry = settings.joints[settings.active_joint]
            kind = entry.control.get("uipc_joint_type", "unknown") if entry.control else "unknown"
            limits = entry.control.get("uipc_joint_limits") if entry.control else None
            if limits is not None:
                layout.label(text=f"{entry.name} limits: {math.degrees(limits[0]):.2f} to {math.degrees(limits[1]):.2f} deg")
            elif kind != "revolute":
                layout.label(text="Fixed joint" if kind == "fixed" else "Unknown legacy joint type: refresh/re-import")
            velocity = entry.control.get("uipc_joint_velocity_limit") if entry.control else None
            if velocity is not None:
                layout.label(text=f"URDF speed limit: {math.degrees(velocity):.2f} deg/s")
        row = layout.row(align=True)
        row.operator("uipc.robot_preview")
        row.operator("uipc.robot_key", text="Key All Joints")
        layout.operator("uipc.robot_key", text="Key Root + Joints").include_root = True
        layout.prop_search(settings, "pose_name", settings, "poses", text="Joint Pose")
        row = layout.row(align=True)
        row.operator("uipc.robot_pose_save", text="Save / Replace Pose")
        row.operator("uipc.robot_pose_apply", text="Apply Pose")
        layout.prop(settings, "speed_review")
        layout.prop(settings, "acceleration_review")
        layout.operator("uipc.robot_review")
        if settings.review:
            review = json.loads(settings.review)
            layout.label(text=f"Last review: frames {review['frame_start']}-{review['frame_end']}; rerun after curve edits")
            rows = review["rows"]
            layout.label(text=f"{len(rows)} joints checked; {sum(bool(r['warning']) for r in rows)} need review")
            layout.prop(settings, "review_detail")
            visible = rows if settings.review_detail else sorted(rows, key=lambda r: (not r["warning"], -r["max_speed_deg_s"]))[:4]
            for row in visible:
                layout.label(text=f"{row['name']}: {row['max_speed_deg_s']:.2f} deg/s, {row['max_acceleration_deg_s2']:.2f} deg/s^2",
                             icon="ERROR" if row["warning"] else "INFO")
        layout.label(text="Review samples joint angles, not root or end-effector speeds")


ROBOT_CLASSES = (UIPCJointControl, UIPCRobotPose, UIPCRobotSettings, UIPC_OT_robot_refresh,
                 UIPC_OT_robot_preview, UIPC_OT_robot_pose_save, UIPC_OT_robot_pose_apply,
                 UIPC_OT_robot_key, UIPC_OT_robot_review, UIPC_UL_joint_controls, UIPC_PT_robot_controls)
