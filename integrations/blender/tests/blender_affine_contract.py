# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Reject lossy TRS-only ABD playback designs; MDD preserves affine samples."""
from pathlib import Path
import sys
import tempfile

import bpy
from mathutils import Matrix
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from protocol import MDDWriter
sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_integration import evaluated

rest = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                 [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]], dtype=float) * .1
affine = np.eye(4)
affine[0,1], affine[1,2], affine[2,2] = .02, -.01, 1.03
affine[:3,3] = [.1,.2,.3]
points = rest @ affine[:3,:3].T + affine[:3,3]
loc, rot, scale = Matrix(affine).decompose()
trs = np.array(Matrix.LocRotScale(loc, rot, scale))
loss = float(np.abs(points - (rest @ trs[:3,:3].T + trs[:3,3])).max())
assert loss > .001, loss  # A legitimate full-affine change is lost, even at 2% shear.
mesh = bpy.data.meshes.new("Affine")
mesh.from_pydata(rest, [], [(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)])
obj = bpy.data.objects.new("Affine", mesh)
bpy.context.scene.collection.objects.link(obj)
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / "affine.mdd"
    writer = MDDWriter(path, 2, len(rest), 30)
    writer.append(rest)
    writer.append(points)
    writer.close(commit=True)
    modifier = obj.modifiers.new("Native affine output", "MESH_CACHE")
    modifier.cache_format, modifier.filepath = "MDD", str(path)
    modifier.time_mode, modifier.play_mode = "FRAME", "SCENE"
    modifier.frame_start, modifier.interpolation = 7, "LINEAR"
    modifier.forward_axis, modifier.up_axis = "POS_Y", "POS_Z"
    for frame, fraction, expected in ((7,0,rest), (8,0,points), (7,.5,(rest+points)/2)):
        np.testing.assert_allclose(evaluated(obj, bpy.context.scene, frame, fraction), expected, atol=3e-8)
print(f"PASS: full affine MDD including subframes; TRS-only maximum error = {loss:.9f} m")
