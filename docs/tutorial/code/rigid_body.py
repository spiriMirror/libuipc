"""Minimal pure-affine-body scene used by the rigid-body tutorial."""

from pathlib import Path

import numpy as np

from uipc import Logger, Transform, Vector3, view
from uipc.core import Engine, Scene, World
from uipc.geometry import ground, label_surface, label_triangle_orient, tetmesh
from uipc.constitution import AffineBodyConstitution
from uipc.unit import GPa, MPa


Logger.set_level(Logger.Level.Warn)
workspace = Path("output/docs/rigid_body")
workspace.mkdir(parents=True, exist_ok=True)

engine = Engine("cuda", str(workspace))
world = World(engine)

config = Scene.default_config()
config["dt"] = 0.01
config["gravity"] = [[0.0], [-9.8], [0.0]]
scene = Scene(config)

# Contact is global, while friction/resistance live in the pairwise table.
scene.contact_tabular().default_model(0.4, 1.0 * GPa)
default_contact = scene.contact_tabular().default_element()

# A closed tetrahedral volume is one affine body instance.
s3 = np.sqrt(3.0) / 2.0
vertices = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-s3, 0.0, -0.5], [s3, 0.0, -0.5]]
) * 0.3
tetrahedra = np.array([[0, 1, 2, 3]])
body = tetmesh(vertices, tetrahedra)
label_surface(body)
label_triangle_orient(body)

abd = AffineBodyConstitution()
abd.apply_to(body, kappa=100.0 * MPa, mass_density=1000.0)
default_contact.apply_to(body)

# ABD state is per instance. Move the instance, not its local rest vertices.
transform = Transform.Identity()
transform.translate(Vector3.Values([0.0, 1.2, 0.0]))
view(body.transforms())[0] = transform.matrix()

scene.objects().create("falling_body").geometries().create(body)

floor = ground(0.0)
default_contact.apply_to(floor)
scene.objects().create("floor").geometries().create(floor)

world.init(scene)
assert world.is_valid()
for _ in range(10):
    world.advance()
    world.retrieve()
assert world.is_valid()
print(f"rigid-body scene reached frame {world.frame()}")
