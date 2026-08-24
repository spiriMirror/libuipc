"""Minimal pinned-cloth scene used by the cloth tutorial."""

from pathlib import Path

import numpy as np

import uipc.builtin as builtin
from uipc import Logger, view
from uipc.core import Engine, Scene, World
from uipc.geometry import ground, label_surface, trimesh
from uipc.constitution import (
    DiscreteShellBending,
    ElasticModuli2D,
    StrainLimitingBaraffWitkinShell,
)
from uipc.unit import MPa, kPa


Logger.set_level(Logger.Level.Warn)
workspace = Path("output/docs/cloth")
workspace.mkdir(parents=True, exist_ok=True)

engine = Engine("cuda", str(workspace))
world = World(engine)

config = Scene.default_config()
config["dt"] = 0.01
config["gravity"] = [[0.0], [-9.8], [0.0]]
config["contact"]["d_hat"] = 0.005
scene = Scene(config)

scene.contact_tabular().default_model(0.3, 100.0 * MPa)
default_contact = scene.contact_tabular().default_element()

# Two triangles are enough to expose stretch, shear, one bending edge, pins,
# and contact. Production cloth should use a suitably resolved surface mesh.
vertices = np.array(
    [[-0.5, 1.5, 0.0], [0.5, 1.5, 0.0], [-0.5, 0.5, 0.0], [0.5, 0.5, 0.0]]
)
triangles = np.array([[0, 2, 1], [1, 2, 3]])
cloth = trimesh(vertices, triangles)
label_surface(cloth)

stretch = ElasticModuli2D.youngs_poisson(50.0 * kPa, 0.40)
shear = ElasticModuli2D.youngs_poisson(0.5 * kPa, 0.40)
StrainLimitingBaraffWitkinShell().apply_to(
    cloth,
    stretch_moduli=stretch,
    shear_moduli=shear,
    mass_density=200.0,
    thickness=0.001,
    strain_rate=100.0,
)

# Apply membrane first: the formula overload reads its vertex thickness.
DiscreteShellBending().apply_to(cloth, 50.0 * kPa, 0.40)
default_contact.apply_to(cloth)

is_fixed = cloth.vertices().find(builtin.is_fixed)
fixed = view(is_fixed)
fixed[0] = 1
fixed[1] = 1

scene.objects().create("cloth").geometries().create(cloth)

floor = ground(0.0)
default_contact.apply_to(floor)
scene.objects().create("floor").geometries().create(floor)

world.init(scene)
assert world.is_valid()
for _ in range(10):
    world.advance()
    world.retrieve()
assert world.is_valid()
print(f"cloth scene reached frame {world.frame()}")
