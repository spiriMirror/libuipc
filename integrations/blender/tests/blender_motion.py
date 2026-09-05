# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Real Blender/CUDA check for timeline-driven ABD bodies and cache signatures."""

import argparse
import importlib
from pathlib import Path
import struct
import sys

import bpy
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--module", default="libuipc_blender")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    args.output = args.output.resolve()
    if args.module == "libuipc_blender":
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    addon = importlib.import_module(args.module)
    if not hasattr(bpy.types.Scene, "uipc_settings"):
        addon.register()
    demo = importlib.import_module(args.module + ".demo")
    scene = bpy.context.scene
    for obj in list(scene.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    scene.frame_start, scene.frame_end, scene.render.fps = 1, 81, 30
    scene.uipc_settings.python_executable = args.python
    scene.uipc_settings.cache_directory = str(args.output)
    scene.uipc_settings.substeps = 2
    body = demo.box(scene, "Servo body", (0, 0, 0.2), (0.1, 0.1, 0.1), "RIGID")
    target = bpy.data.objects.new("Controller", None)
    scene.collection.objects.link(target)
    body.uipc_body.driven = True
    body.uipc_body.drive_target = target
    for frame, z in ((1, 0), (50, 0), (80, 0.20), (81, 0.20)):
        target.location.z = z
        target.keyframe_insert(data_path="location", index=2, frame=frame)
    for curve in target.animation_data.action.fcurves:
        for key in curve.keyframe_points:
            key.interpolation = "LINEAR"
    scene.frame_set(1)
    material0 = addon.bridge.object_material(body)
    scene.frame_set(70)
    assert (
        addon.bridge.object_material(body) == material0
    ), "Signature depends on playback frame"
    scene.frame_set(1)
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    with (directory / "object_0000.mdd").open("rb") as stream:
        frames, vertices = struct.unpack(">ii", stream.read(8))
        stream.read(4 * frames)
        points = np.frombuffer(stream.read(), dtype=">f4").reshape(frames, vertices, 3)
    held = float(np.abs(points[:50] - points[0]).max())
    displacement = points[-1].mean(axis=0) - points[0].mean(axis=0)
    assert held < 1e-5, held
    np.testing.assert_allclose(displacement, [0, 0, 0.2], atol=2e-5)
    scene.frame_set(60)
    addon.bridge.check_cache(scene)
    target.animation_data.action.fcurves[0].keyframe_points[2].co.y += 0.01
    try:
        addon.bridge.check_cache(scene)
    except ValueError:
        pass
    else:
        raise AssertionError("Edited motion curve did not invalidate cache")
    print(
        "MOTION_TEST_PASSED", {"held_error": held, "translation": displacement.tolist()}
    )


if __name__ == "__main__":
    main()
