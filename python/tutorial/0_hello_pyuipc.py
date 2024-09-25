from pyuipc_loader import pyuipc
from pyuipc import Vector3
from pyuipc.geometry import *
from pyuipc.world import World, Scene, SceneIO
from pyuipc.constitution import AffineBodyConstitution
from pyuipc.engine import Engine
from pyuipc import builtin
from asset import AssetDir

import numpy as np

output_path = AssetDir.output_path(__file__)

engine = Engine("cuda", output_path)
world = World(engine)

config = Scene.default_config()
config['dt'] = 0.01
config['gravity'] = (Vector3.UnitY() * -9.8).tolist()
scene = Scene(config)

# create constituiton
abd = AffineBodyConstitution()

# friction ratio and contact resistance
scene.contact_tabular().default_model(0.05, 1e9)
default_element = scene.contact_tabular().default_element()

# create a regular tetrahedron
Vs = np.array([[0,1,0],
               [0,0,1],
               [-np.sqrt(3)/2, 0, -0.5],
               [np.sqrt(3)/2, 0, -0.5]])

Ts = np.array([[0,1,2,3]])

# setup a base mesh to reduce the later work
base_mesh = tetmesh(Vs, Ts)

# apply the constitution to the base mesh
abd.apply_to(base_mesh, 1e8)
# apply the contact model to the base mesh
default_element.apply_to(base_mesh)

# label the surface, enable the contact
label_surface(base_mesh)
# label the triangle orientation to export the correct surface mesh
label_triangle_orient(base_mesh)

mesh1 = base_mesh.copy()
pos_view = view(mesh1.positions())
# move the mesh up for 1 unit
pos_view += Vector3.UnitY() * 1.2

mesh2 = base_mesh.copy()
is_fixed = mesh2.instances().find(builtin.is_fixed)
is_fixed_view = view(is_fixed)
is_fixed_view[:] = 1

object1 = scene.objects().create("upper_tet")
object1.geometries().create(mesh1)

object2 = scene.objects().create("lower_tet")
object2.geometries().create(mesh2)

world.init(scene)

sio = SceneIO(scene)
sio.write_surface(f"{output_path}/scene_surface{world.frame()}.obj")

while world.frame() < 100:
    world.advance()
    world.retrieve()
    sio.write_surface(f"{output_path}/scene_surface{world.frame()}.obj")