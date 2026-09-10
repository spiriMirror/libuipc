# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Factory Blender check: independent RNA values survive save/reopen unchanged."""

import importlib
from pathlib import Path
import sys
import tempfile

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
obj = bpy.context.object
obj.uipc_body.role = "CLOTH"
obj.uipc_body.poisson = 0.37
for field in ("stretch_poisson", "shear_poisson", "bending_poisson"):
    assert abs(getattr(obj.uipc_body, field) - 0.37) < 1e-6
obj.uipc_body.stretch_poisson = 0.4
obj.uipc_body.shear_poisson = 0.45
obj.uipc_body.bending_poisson = 0.2
expected = addon.bridge.object_material(obj)
assert abs(expected["stretch_poisson"] - 0.4) < 1e-6
assert abs(expected["shear_poisson"] - 0.45) < 1e-6
assert abs(expected["bending_poisson"] - 0.2) < 1e-6
with tempfile.TemporaryDirectory(prefix="uipc-materials-") as directory:
    path = str(Path(directory) / "materials.blend")
    bpy.ops.wm.save_as_mainfile(filepath=path)
    bpy.ops.wm.open_mainfile(filepath=path)
    assert addon.bridge.object_material(bpy.context.object) == expected
addon.unregister()
print("PASS: legacy inheritance, independent edits, schema export and save/reopen")
