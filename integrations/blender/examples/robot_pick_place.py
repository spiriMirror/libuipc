# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Sample 87 robot hand picks a free apple from the current dining scene."""

import argparse
import importlib
import json
from pathlib import Path
import struct
import sys
import time
import zipfile

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def read_mdd(path):
    with Path(path).open("rb") as stream:
        frames, vertices = struct.unpack(">ii", stream.read(8))
        stream.read(4 * frames)
        data = (
            np.frombuffer(stream.read(), dtype=">f4")
            .reshape(frames, vertices, 3)
            .astype(float)
        )
    if not np.isfinite(data).all():
        raise AssertionError(f"Nonfinite trajectory: {path}")
    return data


def world_points(obj, local):
    matrix = np.array(obj.matrix_world)
    return local @ matrix[:3, :3].T + matrix[:3, 3]


def linear_keys(obj):
    if obj.animation_data and obj.animation_data.action:
        for curve in obj.animation_data.action.fcurves:
            for key in curve.keyframe_points:
                key.interpolation = "LINEAR"


def build(args, addon):
    bpy.ops.wm.open_mainfile(filepath=str(args.source))
    scene = bpy.context.scene
    addon.bridge.check_cache(scene)
    apple = scene.objects["06 Apple 5"]
    source_cache = Path(
        bpy.path.abspath(apple.modifiers[addon.protocol.MODIFIER_NAME].filepath)
    )
    center = world_points(apple, read_mdd(source_cache)[149]).mean(axis=0)
    center[2] += 0.003
    addon.bridge.detach_cache(scene)
    scene.frame_start, scene.frame_end = 1, args.frames
    scene.uipc_settings.python_executable = args.python
    scene.uipc_settings.cache_directory = str(args.output / "cache")
    scene.frame_set(1)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "robot_pick_place.blend"))
    assert bpy.ops.uipc.import_robot(filepath=str(args.urdf), blocking=True) == {
        "FINISHED"
    }
    root = bpy.context.view_layer.objects.active
    joints = {
        name: scene.objects[value]
        for name, value in json.loads(root["uipc_robot_joints"]).items()
    }
    targets = {
        name: scene.objects[value]
        for name, value in json.loads(root["uipc_robot_targets"]).items()
    }
    links = {
        name: scene.objects[value]
        for name, value in json.loads(root["uipc_robot_links"]).items()
    }
    plan = json.loads(args.plan.read_text())
    grasp_local = np.array(plan["grasp_center_hand"])
    contact_root = center - grasp_local
    destination = np.array([-0.22, -0.015, 0.810])
    destination_root = destination - grasp_local
    stages = [
        (1, contact_root + [0, 0.25, 0.28], "open"),
        (50, contact_root + [0, 0.25, 0.28], "open"),
        (110, contact_root + [0, 0, 0.13], "open"),
        (150, contact_root, "open"),
        (195, contact_root, "closed"),
        (250, contact_root + [0, 0, 0.22], "closed"),
        (325, destination_root + [0, 0, 0.22], "closed"),
        (380, destination_root, "closed"),
        (420, destination_root, "open"),
        (465, destination_root + [0, 0, 0.22], "open"),
        (500, destination_root + [0, 0.20, 0.25], "open"),
    ]
    for frame, position, pose in stages:
        root.location = position
        root.keyframe_insert(data_path="location", frame=frame)
        for name, control in joints.items():
            angle = plan[pose][name]
            limits = control.get("uipc_joint_limits")
            if limits is not None and not limits[0] - 1e-7 <= angle <= limits[1] + 1e-7:
                raise ValueError(f"Joint {name} exceeds its URDF limits")
            control.rotation_axis_angle[0] = angle
            control.keyframe_insert(
                data_path="rotation_axis_angle", index=0, frame=frame
            )
    linear_keys(root)
    for control in joints.values():
        linear_keys(control)
    scene.frame_set(1)
    deps = scene.view_layers[0].depsgraph
    deps.update()
    for name, obj in links.items():
        obj.matrix_world = targets[name].evaluated_get(deps).matrix_world.copy()
    for name, frame in (
        ("Hold", 1),
        ("Approach", 51),
        ("Descend", 111),
        ("Close", 151),
        ("Lift", 196),
        ("Transfer", 251),
        ("Lower", 326),
        ("Release", 381),
        ("Retreat", 421),
    ):
        scene.timeline_markers.new(name, frame=frame)
    scene["robot_pick_target"] = "06 Apple 5"
    scene["robot_pick_destination"] = destination.tolist()
    scene["robot_pick_start_center"] = center.tolist()
    camera_data = bpy.data.cameras.new("Robot action camera")
    camera = bpy.data.objects.new(camera_data.name, camera_data)
    scene.collection.objects.link(camera)
    camera.location = (0.46, -0.50, 1.50)
    camera.rotation_euler = (
        (Vector((-0.20, 0.27, 1.0)) - camera.location)
        .to_track_quat("-Z", "Y")
        .to_euler()
    )
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 1.50
    scene.camera = camera
    scene.render.resolution_x, scene.render.resolution_y = 1280, 1000
    scene.render.resolution_percentage = 100
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "robot_pick_place.blend"))
    return scene


def inspect_result(scene, addon, args):
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    arrays = {}
    for index, entry in enumerate(request["objects"]):
        if entry["material"]["role"] == "STATIC":
            continue
        obj = scene.objects[entry["name"]]
        arrays[obj.name] = world_points(
            obj, read_mdd(directory / f"object_{index:04d}.mdd")
        )
    apple = arrays["06 Apple 5"]
    centers = apple.mean(axis=1)
    phase = {
        str(frame): {
            "center": centers[min(frame, len(centers)) - 1].tolist(),
            "min_z": float(apple[min(frame, len(centers)) - 1, :, 2].min()),
        }
        for frame in (1, 50, 150, 195, 250, 325, 380, 420, 465, 500)
        if frame <= len(apple)
    }
    hold = max(
        float(abs(points[: min(50, len(points))] - points[0]).max())
        for name, points in arrays.items()
        if name.startswith("Robot link ")
    )
    output = {
        "frames": len(apple),
        "robot_held_first_50_frames_max_error_m": hold,
        "apple_peak_center_height_m": float(centers[49:, 2].max()),
        "apple_phases": phase,
        "apple_final_center": centers[-1].tolist(),
        "apple_final_min_z": float(apple[-1, :, 2].min()),
        "apple_final_rms_speed_m_s": float(
            np.sqrt(
                np.mean(
                    np.sum(
                        (np.diff(apple[-16:], axis=0) * scene.render.fps) ** 2, axis=2
                    )
                )
            )
        ),
        "apple_has_no_constraints": not scene.objects["06 Apple 5"].uipc_body.driven,
        "robot_native_fk_error": max(
            float(o.get("uipc_native_fk_error", 0)) for o in scene.objects
        ),
    }
    (args.output / "pick_place_diagnostics.json").write_text(
        json.dumps(output, indent=2), encoding="utf-8"
    )
    print("PICK_PLACE_DIAGNOSTICS " + json.dumps(output), flush=True)
    np.savez_compressed(args.output / "trajectory_summary.npz", apple_centers=centers)
    return arrays, output


def validate_pick_place(scene, addon, args, arrays, report):
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    triangles = {}
    fixed = {}
    materials = {}
    for index, entry in enumerate(request["objects"]):
        name = entry["name"]
        materials[name] = entry["material"]
        with np.load(directory / f"input_{index:04d}.npz", allow_pickle=False) as data:
            triangles[name] = data["triangles"].copy()
            if name not in arrays:
                fixed[name] = (
                    data["vertices"] @ data["matrix"][:3, :3].T + data["matrix"][:3, 3]
                )
    names = list(materials)
    static_trees = {
        n: BVHTree.FromPolygons(
            [Vector(p) for p in v], triangles[n].tolist(), all_triangles=True
        )
        for n, v in fixed.items()
    }
    for frame in range(scene.frame_end - scene.frame_start + 1):
        trees = dict(static_trees)
        for name, points in arrays.items():
            trees[name] = BVHTree.FromPolygons(
                [Vector(p) for p in points[frame]],
                triangles[name].tolist(),
                all_triangles=True,
            )
        for i, a in enumerate(names):
            ga = materials[a].get("drive", {}).get("group")
            for b in names[i + 1 :]:
                gb = materials[b].get("drive", {}).get("group")
                if ga and ga == gb:
                    continue  # Same explicit robot-internal mask as sample 87.
                hits = trees[a].overlap(trees[b])
                if hits:
                    raise AssertionError(
                        f"Inter-object crossing at frame {frame+1}: {a}/{b}, {len(hits)} triangle pairs"
                    )
        if frame % 100 == 0:
            print("ROBOT_CONTACT_CHECK", frame + 1, flush=True)
    target = "06 Apple 5"
    apple = arrays[target]
    centers = apple.mean(axis=1)
    assert (
        report["robot_held_first_50_frames_max_error_m"] < 1e-5
    ), "Robot moved during the hold"
    assert (
        centers[249, 2] - centers[149, 2] > 0.15
    ), "Apple was not lifted out of the bowl"
    assert (
        np.linalg.norm(centers[324, :2] - centers[149, :2]) > 0.35
    ), "Apple was not transported"
    destination = np.array(scene["robot_pick_destination"])
    assert (
        np.linalg.norm(centers[-1, :2] - destination[:2]) < 0.04
    ), "Apple missed the requested tabletop region"
    assert (
        0.755 < float(apple[-1, :, 2].min()) < 0.775
    ), "Apple is not resting at tabletop height"
    assert (
        report["apple_final_rms_speed_m_s"] < 0.01
    ), "Apple has not settled after release"
    assert report["apple_has_no_constraints"], "The apple must be free, not driven"

    def distance(a, va, b, vb):
        ta = BVHTree.FromPolygons(
            [Vector(p) for p in va], triangles[a].tolist(), all_triangles=True
        )
        tb = BVHTree.FromPolygons(
            [Vector(p) for p in vb], triangles[b].tolist(), all_triangles=True
        )
        return min(
            min(tb.find_nearest(Vector(p))[3] for p in va),
            min(ta.find_nearest(Vector(p))[3] for p in vb),
        )

    contacts = {}
    robot = [n for n in arrays if n.startswith("Robot link ")]
    for frame in (250, 325):
        contacts[str(frame)] = {
            name: distance(target, apple[frame - 1], name, arrays[name][frame - 1])
            for name in robot
        }
        close = [name for name, value in contacts[str(frame)].items() if value < 0.0015]
        assert len(close) >= 2, f"No multipoint robotic grasp at frame {frame}"
    release = min(distance(target, apple[-1], name, arrays[name][-1]) for name in robot)
    cloth_gap = distance(
        target,
        apple[-1],
        "01 White linen tablecloth",
        arrays["01 White linen tablecloth"][-1],
    )
    assert release > 0.02, "The hand did not release and withdraw"
    assert cloth_gap < 0.002, "The placed apple has no cloth support"
    for i in range(1, 5):
        other = arrays[f"06 Apple {i}"][-1].mean(axis=0)
        assert (
            np.linalg.norm(other[:2] - [-0.22, 0.47]) < 0.14 and other[2] > 0.765
        ), "Another apple escaped the bowl"
    # Native playback: compare every base vertex, including all robot links.
    maximum = 0.0
    flags = []
    for obj in scene.objects:
        for modifier in obj.modifiers:
            if modifier.type != "MESH_CACHE":
                flags.append((modifier, modifier.show_viewport))
                modifier.show_viewport = False
    try:
        for frame in (1, 50, 195, 250, 325, 420, 500):
            scene.frame_set(frame)
            deps = scene.view_layers[0].depsgraph
            deps.update()
            for name, world in arrays.items():
                obj = scene.objects[name].evaluated_get(deps)
                mesh = obj.to_mesh()
                try:
                    points = np.array([v.co[:] for v in mesh.vertices])
                    matrix = np.array(obj.matrix_world)
                    points = points @ matrix[:3, :3].T + matrix[:3, 3]
                    maximum = max(maximum, float(abs(points - world[frame - 1]).max()))
                finally:
                    obj.to_mesh_clear()
    finally:
        for modifier, visible in flags:
            modifier.show_viewport = visible
    assert maximum < 2e-6, "Blender playback does not match the solved state"
    report.update(
        success=True,
        checked_inter_object_frames=scene.frame_end,
        internal_robot_contact="disabled as in sample 87; all robot/environment and apple contact enabled",
        grasp_surface_distances_m=contacts,
        final_apple_robot_distance_m=release,
        final_apple_cloth_distance_m=cloth_gap,
        all_vertex_playback_error_m=maximum,
        other_four_apples_remain_in_bowl=True,
    )
    (args.output / "pick_place_validation.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print("ROBOT_PICK_PLACE_VERIFIED", flush=True)


def snapshots(scene, args):
    scene.camera.location = (0.46, -0.50, 1.50)
    scene.camera.rotation_euler = (
        (Vector((-0.20, 0.27, 1.0)) - scene.camera.location)
        .to_track_quat("-Z", "Y")
        .to_euler()
    )
    scene.camera.data.ortho_scale = 1.50
    preferences = bpy.context.preferences.addons["cycles"].preferences
    preferences.compute_device_type = "OPTIX"
    preferences.get_devices()
    for device in preferences.devices:
        device.use = device.type == "OPTIX"
    scene.cycles.device = "GPU"
    scene.cycles.samples = 48
    scene.cycles.denoiser = "OPTIX"
    scene.render.use_persistent_data = True
    for frame in (1, 150, 195, 250, 325, 420, 500):
        if frame > scene.frame_end:
            continue
        scene.frame_set(frame)
        scene.render.filepath = str(args.output / f"pick_place_{frame:04d}.png")
        bpy.ops.render.render(write_still=True)


def package_result(scene, addon, args):
    addon.bridge.check_cache(scene)
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake)).resolve()
    directory.relative_to(args.output)
    scene.uipc_settings.cache_directory = ""
    scene.frame_set(500)
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D":
                space = area.spaces.active
                space.region_3d.view_perspective = "CAMERA"
                space.overlay.show_extras = False
                space.overlay.show_relationship_lines = False
                space.shading.type = "MATERIAL"
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "robot_pick_place.blend"))
    members = {
        args.output / "robot_pick_place.blend",
        args.output / "pick_place_validation.json",
        args.output / "pick_place_diagnostics.json",
        args.output / "trajectory_summary.npz",
    }
    members.update(args.output.glob("pick_place_*.png"))
    members.update(args.output.glob("standalone_playback_validation.json"))
    for obj in scene.objects:
        for modifier in obj.modifiers:
            if modifier.type == "MESH_CACHE":
                path = Path(bpy.path.abspath(modifier.filepath)).resolve()
                path.relative_to(args.output)
                members.add(path)
    members.update(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix in (".json", ".npz", ".log")
    )
    destination = args.output / "robot_pick_place_bundle.zip"
    with zipfile.ZipFile(
        destination, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6
    ) as archive:
        for path in sorted(members):
            archive.write(path, path.relative_to(args.output).as_posix())
        archive.write(
            Path(__file__).with_name("ROBOT_PICK_PLACE.md"), "ROBOT_PICK_PLACE.md"
        )
        archive.write(args.plan, "robot_hand_grasp.json")
        archive.writestr(
            "OPEN_ME.txt",
            "Extract the entire archive before opening robot_pick_place.blend.\n"
            "Frame 1 starts the simulation; frames 1-50 hold the hand.\n"
            "Frame 250 shows the grasped apple in the air; frame 500 shows placement.\n"
            "Spacebar plays the baked motion. Keep cache/ and the stem MDD files beside the blend.\n"
            "Playback works without an addon or CUDA. Install libuipc Physics 0.3 to edit controls/re-bake.\n"
            "The library's current source Python/CUDA runtime is required only for re-baking.\n",
        )
    print("ROBOT_BUNDLE", destination, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument(
        "--plan", type=Path, default=Path(__file__).with_name("robot_hand_grasp.json")
    )
    parser.add_argument("--python", required=True)
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument(
        "--mode",
        choices=("all", "build", "bake", "inspect", "shots", "package"),
        default="all",
    )
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    for key in ("source", "output", "urdf", "plan"):
        setattr(args, key, getattr(args, key).resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    addon = importlib.import_module("libuipc_blender")
    addon.register()
    if args.mode in ("all", "build"):
        scene = build(args, addon)
    else:
        bpy.ops.wm.open_mainfile(filepath=str(args.output / "robot_pick_place.blend"))
        scene = bpy.context.scene
    if args.mode in ("all", "bake"):
        directory = addon.runtime.start(scene)
        print("ROBOT_BAKE " + str(directory), flush=True)
        previous = -1
        while addon.runtime.is_running():
            result = addon.runtime.poll()
            if isinstance(result, dict) and result.get("cancelled"):
                return
            percent = int(scene.uipc_settings.progress * 100)
            if percent != previous:
                print("ROBOT_PROGRESS", percent, flush=True)
                previous = percent
            time.sleep(0.1)
        bpy.ops.wm.save_as_mainfile(
            filepath=str(args.output / "robot_pick_place.blend")
        )
    if args.mode in ("all", "bake", "inspect"):
        arrays, report = inspect_result(scene, addon, args)
        import white_table_setting

        white_table_setting.stems(scene, arrays, addon, args.output)
        white_table_setting.activate_cache(scene, addon)
        if scene.frame_end >= 500:
            validate_pick_place(scene, addon, args, arrays, report)
        scene.frame_set(scene.frame_end)
        bpy.ops.wm.save_as_mainfile(
            filepath=str(args.output / "robot_pick_place.blend")
        )
    if args.mode in ("all", "bake", "inspect", "shots"):
        snapshots(scene, args)
        scene.frame_set(scene.frame_end)
        bpy.ops.wm.save_as_mainfile(
            filepath=str(args.output / "robot_pick_place.blend")
        )
    if args.mode == "package":
        package_result(scene, addon, args)


if __name__ == "__main__":
    main()
