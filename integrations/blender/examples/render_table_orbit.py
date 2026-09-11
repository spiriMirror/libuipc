# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Render every cached simulation frame while a native camera rig orbits once.

Run in background Blender with --factory-startup; no physics is re-simulated.
The source .blend and MDD files remain unchanged. The presentation-only copy
detaches add-on job metadata, keeping native caches and an editable render range.
"""

import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import struct
import sys
import time

import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from render_protocol import png_info, replace_with_retry


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, data):
    temporary = path.with_suffix(".json.partial")
    temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
    temporary.replace(path)


def gpu_settings(scene):
    preferences = bpy.context.preferences.addons["cycles"].preferences
    preferences.compute_device_type = "OPTIX"
    preferences.get_devices()
    devices = [device for device in preferences.devices if device.type == "OPTIX"]
    if not devices:
        raise RuntimeError(
            "This batch requires an OptiX GPU; choose an explicit CPU workflow otherwise"
        )
    for device in preferences.devices:
        device.use = device in devices
    scene.render.engine = "CYCLES"
    scene.cycles.device = "GPU"
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = "OPTIX"
    scene.render.use_persistent_data = True
    scene.render.use_motion_blur = False
    scene.cycles.use_animated_seed = False
    return [device.name for device in devices]


def cache_paths(scene, last_frame):
    result = []
    for obj in scene.objects:
        for modifier in obj.modifiers:
            if modifier.type != "MESH_CACHE":
                continue
            path = Path(bpy.path.abspath(modifier.filepath)).resolve()
            if not modifier.show_render or not modifier.show_viewport:
                raise ValueError(f"Disabled cache: {obj.name}")
            with path.open("rb") as stream:
                frames, vertices = struct.unpack(">ii", stream.read(8))
            if (
                frames < last_frame
                or path.stat().st_size != 8 + 4 * frames + frames * vertices * 12
            ):
                raise ValueError(
                    f"Cache cannot supply the requested frame range: {path}"
                )
            # Absolute while changing .blend locations; remap relative after save.
            modifier.filepath = str(path)
            result.append(
                {
                    "object": obj.name,
                    "path": str(path),
                    "sha256": sha256(path),
                    "frames": frames,
                    "vertices": vertices,
                }
            )
    if len(result) != 22:
        raise ValueError("Expected 17 physical caches and 5 derived apple-stem caches")
    return result


def bounding_corners(scene, frame):
    scene.frame_set(frame)
    deps = bpy.context.evaluated_depsgraph_get()
    deps.update()
    corners = []
    for source in scene.objects:
        if (
            source.type != "MESH"
            or source.hide_render
            or source.name == "Studio floor continuation"
        ):
            continue
        obj = source.evaluated_get(deps)
        corners.extend(obj.matrix_world @ Vector(point) for point in obj.bound_box)
    return np.column_stack((np.array(corners), np.ones(len(corners))))


def create_orbit(scene, first, last):
    camera = scene.camera
    pivot = Vector((0.20, 0.06, 0.47))
    scene.frame_set(first)
    matrix = camera.matrix_world.copy()
    offset = matrix.translation - pivot
    rig = bpy.data.objects.new("Camera orbit - 360 degrees", None)
    scene.collection.objects.link(rig)
    rig.empty_display_type = "PLAIN_AXES"
    rig.empty_display_size = 0.20
    rig.location = pivot
    rig.rotation_mode = "XYZ"
    rig.rotation_euler.z = 0
    rig.keyframe_insert(data_path="rotation_euler", index=2, frame=first)
    rig.rotation_euler.z = 2 * math.pi
    rig.keyframe_insert(data_path="rotation_euler", index=2, frame=last)
    for curve in rig.animation_data.action.fcurves:
        curve.extrapolation = "CONSTANT"
        for key in curve.keyframe_points:
            key.interpolation = "LINEAR"
    camera.parent = rig
    camera.matrix_parent_inverse = Matrix.Identity(4)
    camera.location = offset
    camera.rotation_euler = (-offset).to_track_quat("-Z", "Y").to_euler()
    # Keep one radius/height/lens for the entire orbit. Inspect the actual
    # deformed bounding boxes at EVERY rendered frame before fitting the camera.
    bounds = []
    for frame in range(first, last + 1):
        bounds.append(bounding_corners(scene, frame))
        if (frame - first) % 100 == 0:
            print(f"PREFLIGHT_GEOMETRY {frame}/{last}", flush=True)
    worst = None
    factor = 1.0
    for attempt in range(30):
        camera.location = offset * factor
        minimum, maximum = np.array([np.inf, np.inf]), np.array([-np.inf, -np.inf])
        nearest = np.inf
        for frame, points in zip(range(first, last + 1), bounds):
            scene.frame_set(frame)
            deps = bpy.context.evaluated_depsgraph_get()
            deps.update()
            evaluated = camera.evaluated_get(deps)
            projection = evaluated.calc_matrix_camera(
                deps,
                x=scene.render.resolution_x,
                y=scene.render.resolution_y,
                scale_x=scene.render.pixel_aspect_x,
                scale_y=scene.render.pixel_aspect_y,
            )
            view = np.array(evaluated.matrix_world.inverted())
            clip = points @ (np.array(projection) @ view).T
            ndc = (clip[:, :2] / clip[:, 3:4] + 1) / 2
            minimum = np.minimum(minimum, ndc.min(axis=0))
            maximum = np.maximum(maximum, ndc.max(axis=0))
            nearest = min(nearest, float(clip[:, 3].min()))
        worst = {
            "minimum_image_xy": minimum.tolist(),
            "maximum_image_xy": maximum.tolist(),
            "nearest_camera_depth_m": nearest,
            "margin_fraction": 0.045,
        }
        if (
            nearest > camera.data.clip_start
            and np.all(minimum >= 0.045)
            and np.all(maximum <= 0.955)
        ):
            break
        factor *= 1.04
    else:
        raise RuntimeError(f"Could not frame the complete orbit: {worst}")
    scene.frame_set(first)
    start_matrix = np.array(
        camera.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world
    )
    scene.frame_set(last)
    end_matrix = np.array(
        camera.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world
    )
    endpoint_error = float(abs(start_matrix - end_matrix).max())
    if endpoint_error > 1e-5:
        raise AssertionError(
            f"The orbit did not return to its starting pose: {endpoint_error}"
        )
    scene.frame_set(first)
    return {
        "pivot_m": list(pivot),
        "radius_m": float(np.linalg.norm(np.array(camera.location)[:2])),
        "height_m": float(pivot.z + camera.location.z),
        "fit_scale": factor,
        "degrees": 360,
        "direction": "counterclockwise viewed from +Z",
        "angle_formula": f"2*pi*(frame-{first})/({last}-{first})",
        "endpoint_matrix_max_error": endpoint_error,
        "framing": worst,
    }


def prepare(args):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    addon = importlib.import_module("libuipc_blender")
    if not hasattr(bpy.types.Scene, "uipc_settings"):
        addon.register()
    bpy.ops.wm.open_mainfile(filepath=str(args.source))
    scene = bpy.context.scene
    addon.bridge.check_cache(scene)
    caches = cache_paths(scene, args.last)
    # This is a presentation copy, not a new or modified physics job. Detaching
    # only job metadata prevents the add-on from treating the shorter render
    # range as stale physics and disabling the unchanged native MDD modifiers.
    scene.uipc_settings.last_bake = ""
    scene.uipc_settings.baked_fingerprint = ""
    scene.frame_start, scene.frame_end = args.first, args.last
    scene.frame_step = 1
    scene.render.resolution_x, scene.render.resolution_y = args.width, args.height
    scene.render.resolution_percentage = 100
    scene.cycles.samples = args.samples
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.image_settings.color_depth = "8"
    scene.render.use_file_extension = True
    devices = gpu_settings(scene)
    orbit = create_orbit(scene, args.first, args.last)
    scene.render.filepath = str(args.output / "frames" / "frame_")
    scene["orbit_source_physics"] = str(args.source)
    scene["orbit_note"] = (
        "Presentation-only camera animation; existing libuipc MDD trajectories are unchanged."
    )
    blend = args.output / "table_orbit_0001_0500.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    for obj in scene.objects:
        for modifier in obj.modifiers:
            if modifier.type == "MESH_CACHE":
                modifier.filepath = bpy.path.relpath(modifier.filepath)
    scene.render.filepath = "//frames/frame_"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    manifest = {
        "schema": 1,
        "source_blend": str(args.source),
        "source_sha256": sha256(args.source),
        "first_frame": args.first,
        "last_frame": args.last,
        "frame_count": args.last - args.first + 1,
        "fps": scene.render.fps / scene.render.fps_base,
        "resolution": [args.width, args.height],
        "samples": args.samples,
        "engine": "CYCLES",
        "denoiser": "OPTIX",
        "devices": devices,
        "orbit": orbit,
        "caches": caches,
        "presentation_blend": blend.name,
    }
    write_json(args.output / "orbit_manifest.json", manifest)
    print("ORBIT_PREPARED " + json.dumps(orbit), flush=True)
    return scene, manifest


def verify_sequence(args, manifest):
    files = sorted((args.output / "frames").glob("frame_*.png"))
    expected = [f"frame_{frame:04d}.png" for frame in range(args.first, args.last + 1)]
    if [path.name for path in files] != expected:
        raise AssertionError("The PNG sequence contains missing or extra frames")
    records = []
    for path in files:
        if png_info(path) != (args.width, args.height):
            raise AssertionError(f"Unexpected resolution: {path}")
        records.append(
            {"file": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)}
        )
    if sha256(args.source) != manifest["source_sha256"]:
        raise AssertionError("The original .blend changed during rendering")
    for cache in manifest["caches"]:
        if sha256(Path(cache["path"])) != cache["sha256"]:
            raise AssertionError(f"Physics cache changed: {cache['path']}")
    report = {
        "complete": True,
        "frame_count": len(records),
        "resolution": [args.width, args.height],
        "original_blend_and_all_caches_unchanged": True,
        "all_png_crcs_valid": True,
        "orbit_degrees": 360,
        "endpoint_matrix_max_error": manifest["orbit"]["endpoint_matrix_max_error"],
        "total_bytes": sum(record["bytes"] for record in records),
        "frames": records,
    }
    write_json(args.output / "sequence_validation.json", report)
    print(f"ORBIT_SEQUENCE_VALIDATED {len(records)} PNGs", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--mode", choices=("prepare", "preview", "render", "verify"), default="render"
    )
    parser.add_argument("--first", type=int, default=1)
    parser.add_argument("--last", type=int, default=500)
    parser.add_argument("--width", type=int, default=2000)
    parser.add_argument("--height", type=int, default=1450)
    parser.add_argument("--samples", type=int, default=192)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    args.source, args.output = args.source.resolve(), args.output.resolve()
    if (
        args.first < 1
        or args.last <= args.first
        or min(args.width, args.height, args.samples) <= 0
    ):
        parser.error(
            "Use a positive resolution/sample count and at least two ordered frames"
        )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "frames").mkdir(exist_ok=True)
    existing = args.output / "orbit_manifest.json"
    if existing.exists():
        manifest = json.loads(existing.read_text())
        for key, value in (
            ("source_blend", str(args.source)),
            ("source_sha256", sha256(args.source)),
            ("first_frame", args.first),
            ("last_frame", args.last),
            ("resolution", [args.width, args.height]),
            ("samples", args.samples),
        ):
            if manifest[key] != value:
                raise ValueError(
                    f"Existing job differs in {key}; use another output directory"
                )
        if args.mode == "verify":
            verify_sequence(args, manifest)
            return
        bpy.ops.wm.open_mainfile(
            filepath=str(args.output / manifest["presentation_blend"])
        )
        scene = bpy.context.scene
        gpu_settings(scene)
    else:
        if args.mode == "verify":
            raise ValueError("No prepared orbit job exists")
        scene, manifest = prepare(args)
    if args.mode == "prepare":
        return
    if args.mode == "preview":
        directory = args.output / "previews"
        directory.mkdir(exist_ok=True)
        frames = sorted(
            set(
                round(args.first + (args.last - args.first) * t)
                for t in (0, 0.25, 0.5, 0.75, 1)
            )
        )
        scene.cycles.samples = 32
        scene.render.resolution_percentage = 35
    else:
        directory = args.output / "frames"
        frames = range(args.first, args.last + 1)
    started = time.monotonic()
    for frame in frames:
        final = directory / f"frame_{frame:04d}.png"
        if args.mode == "render" and final.exists():
            if png_info(final) != (args.width, args.height):
                raise ValueError(f"Existing frame has unexpected dimensions: {final}")
            continue
        scene.frame_set(frame)
        temporary = directory / f"pending_{frame:04d}.png"
        scene.render.filepath = str(temporary)
        bpy.ops.render.render(write_still=True)
        size = png_info(temporary)
        expected = (
            (args.width, args.height)
            if args.mode == "render"
            else (int(args.width * 0.35), int(args.height * 0.35))
        )
        if size != expected:
            raise AssertionError(f"Rendered size mismatch: {size} != {expected}")
        replace_with_retry(temporary, final)
        elapsed = time.monotonic() - started
        write_json(
            args.output / "render_progress.json",
            {
                "state": "rendering",
                "mode": args.mode,
                "last_frame": frame,
                "first": args.first,
                "last": args.last,
                "elapsed_seconds": elapsed,
            },
        )
        print(f"ORBIT_FRAME {frame}/{args.last} {elapsed:.1f}s {final}", flush=True)
    if args.mode == "render":
        verify_sequence(args, manifest)
        write_json(
            args.output / "render_progress.json",
            {
                "state": "complete",
                "mode": args.mode,
                "last_frame": args.last,
                "first": args.first,
                "last": args.last,
                "elapsed_seconds": time.monotonic() - started,
            },
        )


if __name__ == "__main__":
    main()
