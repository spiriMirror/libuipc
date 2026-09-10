# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Run with the external pyuipc Python to check real constitutive attributes."""

from pathlib import Path
import sys

import numpy as np
from uipc.geometry import trimesh
from uipc.constitution import StrainLimitingBaraffWitkinShell, DiscreteShellBending, ElasticModuli2D

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from worker import apply_cloth_material
from materials import cloth_stiffness

material = dict(role="CLOTH", stretch=1e4, shear=10, bending=1e4, poisson=.49,
                stretch_poisson=.4, shear_poisson=.45, bending_poisson=.2,
                thickness=.0005, density=200, strain_rate=100)
mesh = trimesh(np.array([[0., 0, 0], [1., 0, 0], [0., 1, 0], [1., 1, 0]]),
               np.array([[0, 1, 2], [1, 3, 2]]))
apply_cloth_material(mesh, material, StrainLimitingBaraffWitkinShell(),
                     DiscreteShellBending(), ElasticModuli2D)
expected = cloth_stiffness(material)
np.testing.assert_allclose(mesh.triangles().find("lambda").view(), expected["stretch"])
np.testing.assert_allclose(mesh.triangles().find("mu").view(), expected["shear"])
np.testing.assert_allclose(mesh.edges().find("bending_stiffness").view(), expected["bending"])
print("PASS: native triangle/edge stiffness matches independent E/nu", expected)
