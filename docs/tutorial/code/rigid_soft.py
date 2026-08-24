"""Minimal ABD/FEM coupled-contact scene used by the coupling tutorial."""

from pathlib import Path

import numpy as np

from uipc import Logger, Transform, Vector3, view
from uipc.core import Engine, Scene, World
from uipc.geometry import ground, label_surface, label_triangle_orient, tetmesh
from uipc.constitution import AffineBodyConstitution, ElasticModuli, StableNeoHookean
from uipc.unit import GPa, MPa, kPa


Logger.set_level(Logger.Level.Warn)
workspace = Path("output/docs/rigid_soft")
workspace.mkdir(parents=True, exist_ok=True)

engine = Engine("cuda", str(workspace))
world = World(engine)

config = Scene.default_config()
config["dt"] = 0.01
config["gravity"] = [[0.0], [-9.8], [0.0]]
scene = Scene(config)

table = scene.contact_tabular()
rigid_contact = table.default_element()
soft_contact = table.create("soft")
table.default_model(0.3, 1.0 * GPa)
table.insert(rigid_contact, soft_contact, 0.5, 300.0 * MPa)

s3 = np.sqrt(3.0) / 2.0
vertices = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-s3, 0.0, -0.5], [s3, 0.0, -0.5]]
) * 0.3
tetrahedra = np.array([[0, 1, 2, 3]])
base = tetmesh(vertices, tetrahedra)
label_surface(base)
label_triangle_orient(base)

# Deformable body: every vertex owns three FEM degrees of freedom.
soft = base.copy()
view(soft.positions())[:] += Vector3.Values([0.0, 0.25, 0.0])
moduli = ElasticModuli.youngs_poisson(100.0 * kPa, 0.45)
StableNeoHookean().apply_to(soft, moduli, mass_density=1000.0)
soft_contact.apply_to(soft)
scene.objects().create("soft_body").geometries().create(soft)

# Affine body: the mesh is local geometry and its instance owns 12 ABD DOFs.
rigid = base.copy()
AffineBodyConstitution().apply_to(rigid, 100.0 * MPa, mass_density=1000.0)
rigid_contact.apply_to(rigid)
transform = Transform.Identity()
transform.translate(Vector3.Values([0.0, 1.0, 0.0]))
view(rigid.transforms())[0] = transform.matrix()
scene.objects().create("rigid_body").geometries().create(rigid)

floor = ground(0.0)
rigid_contact.apply_to(floor)
scene.objects().create("floor").geometries().create(floor)

world.init(scene)
assert world.is_valid()
for _ in range(15):
    world.advance()
    world.retrieve()
assert world.is_valid()
print(f"rigid-soft scene reached frame {world.frame()}")
