# SPDX-License-Identifier: Apache-2.0
"""Run with real Blender, its depsgraph/render pipeline, and a CUDA worker.

blender --background --factory-startup --python-exit-code 1 --python <this file>
  -- --addon-parent <integrations/blender> --python <external python> --output <dir>
"""

import argparse
import importlib
import json
from pathlib import Path
import struct
import sys
import time

import bpy
import numpy as np


def evaluated(obj, scene, frame, subframe=0.0):
    scene.frame_set(frame, subframe=subframe)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    depsgraph.update()
    result = obj.evaluated_get(depsgraph)
    mesh = result.to_mesh()
    try:
        points = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", points)
        return points.reshape(-1, 3).copy()
    finally:
        result.to_mesh_clear()


def read_mdd(path):
    with Path(path).open("rb") as stream:
        frames, vertices = struct.unpack(">ii", stream.read(8))
        times = np.frombuffer(stream.read(4 * frames), dtype=">f4")
        points = np.frombuffer(stream.read(), dtype=">f4").reshape(frames, vertices, 3)
    return times, points


def local_vertices(obj):
    return np.array([v.co[:] for v in obj.data.vertices])


def result_arrays(scene):
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    result = json.loads((directory / "result.json").read_text())
    data = {}
    for obj in result["objects"]:
        name = request["objects"][obj["index"]]["name"]
        times, points = read_mdd(directory / f"object_{obj['index']:04d}.mdd")
        data[name] = points
    return directory, request, result, data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--addon-parent", type=Path)
    parser.add_argument("--module", default="libuipc_blender")
    parser.add_argument("--python", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.addon_parent:
        sys.path.insert(0, str(args.addon_parent.resolve()))
    addon = importlib.import_module(args.module)
    if not hasattr(bpy.types.Scene, "uipc_settings"):
        addon.register()
    demo = importlib.import_module(args.module + ".demo")
    checks = []

    scene = demo.create_demo()
    bpy.context.window.scene = scene
    scene.uipc_settings.python_executable = args.python
    scene.uipc_settings.cache_directory = str(args.output / "cache")
    if args.quick:
        scene.frame_end = 3
    initial = {o.name: local_vertices(o) for o in scene.objects if o.type == "MESH"}
    assert bpy.ops.uipc.check_runtime() == {"FINISHED"}
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    checks.append("actual Blender operator -> external pyuipc CUDA -> native MDD modifier")
    directory, request, result, arrays = result_arrays(scene)
    assert result["build_info"]["version"] == "0.0.28", result["build_info"]
    maximum_error = 0.0
    frames = [1, 2, scene.frame_end, max(1, scene.frame_end // 2), 1]
    for frame in frames:
        for name, points in arrays.items():
            actual = evaluated(scene.objects[name], scene, frame)
            error = float(np.max(np.abs(actual - points[frame - 1])))
            maximum_error = max(maximum_error, error)
            assert error < 2e-6, (name, frame, error)
    for name, points in arrays.items():
        actual = evaluated(scene.objects[name], scene, 1, subframe=0.5)
        np.testing.assert_allclose(actual, (points[0] + points[1]) / 2, atol=2e-6)
    for name, points in initial.items():
        np.testing.assert_array_equal(local_vertices(scene.objects[name]), points)
    checks.append("every vertex matches MDD at forward/backward frames and fractional frames; base meshes unchanged")

    cloth = scene.objects["Pinned Cloth"]
    cloth_points = arrays[cloth.name]
    group_index = cloth.vertex_groups[cloth.uipc_body.pin_group].index
    pins = [v.index for v in cloth.data.vertices if any(g.group == group_index for g in v.groups)]
    for frame in range(result["frames"]):
        np.testing.assert_allclose(cloth_points[frame, pins], initial[cloth.name][pins], atol=2e-6)
    if not args.quick:
        assert np.max(initial[cloth.name][:, 2] - cloth_points[-1, :, 2]) > 0.3
        assert np.min(cloth_points[:, :, 2]) >= -2e-4
        inside = (np.abs(cloth_points[-1, :, 0]) < 0.35) & (np.abs(cloth_points[-1, :, 1]) < 0.35)
        assert np.any(inside)
        assert np.min(cloth_points[-1, inside, 2]) >= 0.799
        rigid_world = arrays["Falling ABD"] + np.array(scene.objects["Falling ABD"].location)
        assert np.mean(rigid_world[-1, :, 2]) < 1.6
        assert np.min(rigid_world[:, :, 2]) > -2e-4
        checks.append("61-frame cloth/ABD/fixed contact: pinned vertices fixed, cloth deforms, rigid falls, platform/floor not penetrated")
    assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
    if args.quick:
        print("BLENDER_QUICK_OK", json.dumps(result))
        return

    # Native MDD playback survives saving/reopening without a frame-change callback.
    scene.frame_set(61)
    blend = args.output / "libuipc_demo.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    scene_name = scene.name
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    scene = bpy.data.scenes[scene_name]
    bpy.context.window.scene = scene
    for name, points in arrays.items():
        np.testing.assert_allclose(evaluated(scene.objects[name], scene, 61), points[-1], atol=2e-6)
    assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
    checks.append("saved .blend reopened and every cached vertex revalidated")

    for frame in (1, 61):
        scene.frame_set(frame)
        scene.render.filepath = str(args.output / f"demo_frame_{frame:04d}.png")
        assert bpy.ops.render.render(write_still=True) == {"FINISHED"}
    checks.append("Blender EEVEE renders initial/final cached frames")

    # Physics edits must invalidate cache, including topology changes that retain count.
    cloth = scene.objects["Pinned Cloth"]
    cloth.uipc_body.stretch *= 2
    assert not cloth.modifiers[addon.protocol.MODIFIER_NAME].show_render
    try:
        addon.bridge.check_cache(scene)
        raise AssertionError("material change did not invalidate cache")
    except ValueError:
        pass
    cloth.uipc_body.stretch /= 2
    cloth.data.vertices[0].co.x += 0.05
    try:
        addon.bridge.check_cache(scene)
        raise AssertionError("mesh edit did not invalidate cache")
    except ValueError:
        pass
    cloth.data.vertices[0].co.x -= 0.05
    checks.append("material and base-geometry edits invalidate old bake")

    # A cancelled job may not overwrite the previous bake or attach partial output.
    previous = scene.uipc_settings.last_bake
    cancel_directory = addon.runtime.start(scene)
    addon.runtime.request_cancel()
    started = time.monotonic()
    while addon.runtime.is_running():
        addon.runtime.poll()
        assert time.monotonic() - started < 20
        time.sleep(0.1)
    assert scene.uipc_settings.last_bake == previous
    assert not (cancel_directory / "result.json").exists()
    checks.append("cancelled worker leaves previous cache intact")

    # Nonuniform negative scale and scene units must round-trip without applying transforms twice.
    scaled_scene = bpy.data.scenes.new("Scale Test")
    bpy.context.window.scene = scaled_scene
    scaled_scene.frame_start, scaled_scene.frame_end = 7, 9
    scaled_scene.render.fps = 25
    scaled_scene.unit_settings.scale_length = 0.01
    scaled_scene.uipc_settings.gravity = (0, 0, 0)
    scaled_scene.uipc_settings.python_executable = args.python
    scaled_scene.uipc_settings.cache_directory = str(args.output / "scale_cache")
    obj = demo.box(scaled_scene, "Mirrored Body", (30, -20, 50), (10, 10, 10), "RIGID")
    obj.scale = (-2, 0.5, 1.5)
    obj.rotation_euler = (0.2, 0.3, 0.4)
    original = local_vertices(obj)
    scaled_scene.frame_set(7)
    bpy.context.view_layer.update()
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    for frame in (7, 8, 9, 7):
        np.testing.assert_allclose(evaluated(obj, scaled_scene, frame), original, atol=2e-5)
    checks.append("nonuniform negative scale, rotation, centimeters, non-1 start frame round-trip")

    displacements = {}
    for friction in (0.0, 1.0):
        slide_scene = bpy.data.scenes.new(f"Friction {friction}")
        bpy.context.window.scene = slide_scene
        slide_scene.frame_start, slide_scene.frame_end = 1, 41
        slide_scene.render.fps = 50
        settings = slide_scene.uipc_settings
        settings.python_executable = args.python
        settings.cache_directory = str(args.output / f"friction_{friction}")
        settings.gravity = (4, 0, -9.81)
        settings.friction = friction
        demo.mesh_object(slide_scene, "Floor", [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
                         [(0, 1, 2, 3)], "STATIC")
        cube = demo.box(slide_scene, "Slider", (0, 0, 0.103), (0.2, 0.2, 0.2), "RIGID")
        slide_scene.frame_set(1)
        bpy.context.view_layer.update()
        assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
        _, _, _, points = result_arrays(slide_scene)
        displacements[str(friction)] = float(points[cube.name][-1, :, 0].mean())
        assert float(points[cube.name][:, :, 2].min()) + cube.location.z > -0.0002
    assert displacements["0.0"] > 0.2, displacements
    assert abs(displacements["1.0"]) < 0.1 * displacements["0.0"], displacements
    checks.append("Coulomb friction changes actual simulated sliding: mu=0 versus mu=1")

    # Missing executable must fail without replacing a cache. Use the same public operator.
    bpy.context.window.scene = scaled_scene
    scaled_scene.uipc_settings.python_executable = str(args.output / "missing-python.exe")
    previous = scaled_scene.uipc_settings.last_bake
    try:
        bpy.ops.uipc.bake(blocking=True)
        raise AssertionError("missing interpreter did not fail")
    except RuntimeError:
        pass
    assert scaled_scene.uipc_settings.last_bake == previous
    assert not addon.runtime.is_running()
    scaled_scene.uipc_settings.python_executable = args.python
    # Re-bake a completed world: output gets a fresh directory and restores valid cache.
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    assert scaled_scene.uipc_settings.last_bake != previous
    assert bpy.ops.uipc.detach_cache() == {"FINISHED"}
    np.testing.assert_array_equal(local_vertices(obj), original)
    checks.append("failed launch, rebake, and detach preserve source geometry and old cache files")

    bpy.context.window.scene = scene
    # Restore the already validated demo from disk instead of retaining test edits.
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    summary = {"blender": bpy.app.version_string, "worker": result["build_info"],
               "maximum_cache_vertex_error": maximum_error, "friction_displacements": displacements,
               "checks": checks, "demo": str(blend), "cache": str(directory)}
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("BLENDER_INTEGRATION_OK", json.dumps(summary))


if __name__ == "__main__":
    main()
