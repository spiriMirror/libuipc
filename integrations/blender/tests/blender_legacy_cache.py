# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Read-only validation of an older .blend/cache using the current source addon."""

import argparse
import importlib
from pathlib import Path
import sys

import bpy

parser = argparse.ArgumentParser()
parser.add_argument("--blend", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
bpy.ops.wm.open_mainfile(filepath=str(args.blend.resolve()))
scene = bpy.context.scene
directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
request = addon.protocol.read_json(directory / "request.json")
for frame in (scene.frame_start, (scene.frame_start + scene.frame_end) // 2, scene.frame_end):
    scene.frame_set(frame)
    result = addon.bridge.activate_cache(scene)
    addon.bridge.validate_for_render(scene)
    print("LEGACY_CACHE_VALID", request["schema_version"], frame, result["frames"])
addon.unregister()
