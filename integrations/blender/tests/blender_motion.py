# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Real Blender/CUDA check for timeline-driven ABD bodies and cache signatures."""

import argparse
import importlib
import json
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
    result = json.loads((directory / "result.json").read_text())
    if result["objects"][0].get("encoding") == "AFFINE":
        decode = importlib.import_module(args.module + ".affine").affine_positions
        source = np.array([v.co[:] for v in body.data.vertices])
        points = np.stack([decode(frame, source) for frame in points])
    held = float(np.abs(points[:50] - points[0]).max())
    displacement = points[-1].mean(axis=0) - points[0].mean(axis=0)
    assert held < 1e-5, held
    np.testing.assert_allclose(displacement, [0, 0, 0.2], atol=2e-5)
    scene.frame_set(60)
    addon.bridge.check_cache(scene)
    original_request = json.loads((directory / "request.json").read_text())
    assert "solver_accuracy" not in original_request["settings"]
    scene.uipc_settings.solver_accuracy = "CONVERGED"
    try:
        addon.bridge.check_cache(scene)
    except ValueError:
        pass
    else:
        raise AssertionError("Changed accuracy did not invalidate the old cache")
    scene.uipc_settings.solver_accuracy = "DEFAULT"
    addon.bridge.check_cache(scene)
    scene.uipc_settings.solver_accuracy = "CONVERGED"
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    precise = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    assert precise != directory and (directory / "result.json").exists()
    request = json.loads((precise / "request.json").read_text())
    result = json.loads((precise / "result.json").read_text())
    assert request["settings"]["solver_accuracy"] == "CONVERGED"
    assert result["effective_newton"]["semi_implicit"]["enable"] == 0
    assert result["effective_newton"]["velocity_tol"] == 0.001
    assert result["effective_linear_system"]["tol_rate"] == 1e-6
    addon.bridge.check_cache(scene)
    scene.uipc_settings.solver_accuracy = "CUSTOM"
    scene.uipc_settings.solver_linear_tolerance = "1e-8"
    scene.uipc_settings.solver_velocity_tolerance = "2e-3"
    scene.uipc_settings.solver_newton_max_iter = 64
    scene.uipc_settings.solver_newton_min_iter = 1
    scene.uipc_settings.solver_line_search_max_iter = 16
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    custom = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    result = json.loads((custom / "result.json").read_text())
    assert result["solver_accuracy"] == "CUSTOM"
    assert result["effective_linear_system"]["tol_rate"] == 1e-8
    assert result["effective_newton"]["velocity_tol"] == 0.002
    assert result["effective_newton"]["max_iter"] == 64
    assert result["effective_newton"]["min_iter"] == 1
    assert result["effective_line_search"]["max_iter"] == 16
    scene.uipc_settings.solver_linear_tolerance = "2e-8"
    try:
        addon.bridge.check_cache(scene)
    except ValueError:
        pass
    else:
        raise AssertionError("Edited numerical tolerance did not invalidate cache")
    scene.uipc_settings.solver_linear_tolerance = "nan"
    try:
        addon.bridge.collect_scene(scene)
    except ValueError:
        pass
    else:
        raise AssertionError("Nonfinite custom tolerance was exported")
    scene.uipc_settings.solver_linear_tolerance = "1e-8"
    assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
    saved = args.output / "custom_solver.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(saved))
    bpy.ops.wm.open_mainfile(filepath=str(saved))
    scene = bpy.context.scene
    target = scene.objects["Controller"]
    assert scene.uipc_settings.solver_accuracy == "CUSTOM"
    assert scene.uipc_settings.solver_linear_tolerance == "1e-8"
    addon.bridge.check_cache(scene)
    scene.frame_set(81)
    body = scene.objects["Servo body"]
    modifier = body.modifiers[addon.protocol.MODIFIER_NAME]
    assert modifier.show_viewport and modifier.show_render
    deps = scene.view_layers[0].depsgraph
    deps.update()
    evaluated = body.evaluated_get(deps)
    mesh = evaluated.to_mesh()
    try:
        center = np.mean([evaluated.matrix_world @ v.co for v in mesh.vertices], axis=0)
        assert abs(center[2] - 0.4) < 2e-5, center
    finally:
        evaluated.to_mesh_clear()
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
