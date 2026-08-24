"""Minimal contact-free tetrahedral FEM scene used by the FEM tutorial."""

from pathlib import Path

import numpy as np

from uipc import Logger
from uipc.core import Engine, Scene, World
from uipc.geometry import label_surface, label_triangle_orient, tetmesh
from uipc.constitution import ElasticModuli, StableNeoHookean
from uipc.unit import kPa


Logger.set_level(Logger.Level.Warn)
workspace = Path("output/docs/fem")
workspace.mkdir(parents=True, exist_ok=True)

engine = Engine("cuda", str(workspace))
world = World(engine)

config = Scene.default_config()
config["dt"] = 0.01
config["gravity"] = [[0.0], [-9.8], [0.0]]
config["contact"]["enable"] = False  # isolate the volumetric FEM model
scene = Scene(config)

s3 = np.sqrt(3.0) / 2.0
vertices = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-s3, 0.0, -0.5], [s3, 0.0, -0.5]]
)
tetrahedra = np.array([[0, 1, 2, 3]])
solid = tetmesh(vertices, tetrahedra)
label_surface(solid)
label_triangle_orient(solid)

material = ElasticModuli.youngs_poisson(50.0 * kPa, 0.499)
StableNeoHookean().apply_to(solid, material, mass_density=1000.0)

scene.objects().create("soft_tet").geometries().create(solid)

world.init(scene)
assert world.is_valid()
for _ in range(10):
    world.advance()
    world.retrieve()
assert world.is_valid()
print(f"FEM scene reached frame {world.frame()}")
