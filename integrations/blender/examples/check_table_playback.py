# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Verify a relocated bundle in factory Blender WITHOUT registering the addon."""

import argparse
import json
from pathlib import Path
import struct
import sys

import bpy
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--filename", default="white_table_setting.blend")
    parser.add_argument("--expected-caches", type=int, default=22)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    root = args.directory.resolve()
    assert not hasattr(
        bpy.types.Scene, "uipc_settings"
    ), "Run with --factory-startup and no addon"
    bpy.ops.wm.open_mainfile(filepath=str(root / args.filename))
    scene = bpy.context.scene
    assert (
        scene.frame_current == scene.frame_end
    ), "The file must open at its final frame"
    caches = []
    disabled = []
    for obj in scene.objects:
        for mod in obj.modifiers:
            if mod.type == "MESH_CACHE":
                path = Path(bpy.path.abspath(mod.filepath)).resolve()
                path.relative_to(root)
                assert path.is_file(), f"Missing portable cache: {path}"
                assert (
                    mod.show_render and mod.show_viewport
                ), f"Disabled cache: {obj.name}"
                with path.open("rb") as stream:
                    frames, vertices = struct.unpack(">ii", stream.read(8))
                assert path.stat().st_size == 8 + 4 * frames + 12 * frames * vertices
                points = np.memmap(
                    path,
                    dtype=">f4",
                    mode="r",
                    offset=8 + 4 * frames,
                    shape=(frames, vertices, 3),
                )
                caches.append((obj, points))
            else:
                disabled.append((mod, mod.show_viewport))
                mod.show_viewport = False
    assert len(caches) == args.expected_caches, "Unexpected native cache count"
    maximum = 0.0
    sampled = (1, (scene.frame_end + 1) // 2, scene.frame_end)
    try:
        for frame in sampled:
            scene.frame_set(frame)
            deps = bpy.context.evaluated_depsgraph_get()
            deps.update()
            for source, cache in caches:
                obj = source.evaluated_get(deps)
                mesh = obj.to_mesh()
                try:
                    points = np.array([v.co[:] for v in mesh.vertices])
                    assert points.shape == cache[frame - 1].shape
                    maximum = max(
                        maximum, float(np.abs(points - cache[frame - 1]).max())
                    )
                finally:
                    obj.to_mesh_clear()
    finally:
        for mod, visible in disabled:
            mod.show_viewport = visible
    assert maximum < 2e-6, f"Native playback differs from MDD by {maximum} m"
    scene.frame_set(scene.frame_end)
    report = {
        "addon_registered": False,
        "relocated_cache_objects": len(caches),
        "sampled_frames": sampled,
        "all_vertex_max_error_m": maximum,
        "all_cache_paths_inside_bundle": True,
    }
    (root / "standalone_playback_validation.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    if args.render:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = "OPTIX"
        prefs.get_devices()
        for device in prefs.devices:
            device.use = device.type == "OPTIX"
        scene.cycles.device = "GPU"
        scene.cycles.samples = 32
        scene.render.resolution_percentage = 50
        scene.render.filepath = str(root / "standalone_reopen.png")
        bpy.ops.render.render(write_still=True)
    print("STANDALONE_PLAYBACK " + json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
