from pyuipc_tutorial import * # common import for all tutorial scripts
# =================================================================================================
from pyuipc import Vector3
from pyuipc.engine import Engine
from pyuipc.geometry import tetmesh, label_surface, label_triangle_orient, view
from pyuipc.world import World, Scene, SceneIO
from pyuipc.constitution import AffineBodyConstitution
from pyuipc import builtin
from pyuipc_utils.gui import SceneGUI
import polyscope as ps
import polyscope.imgui as psim
import numpy as np

output_path = AssetDir.output_path(__file__)

engine = Engine("cuda", output_path)
world = World(engine)

config = Scene.default_config()
config['dt'] = 0.01
config['contact']['d_hat'] = 0.01
config['gravity'] = (Vector3.UnitY() * -9.8).tolist()
print(config)
scene = Scene(config)

# friction ratio and contact resistance
scene.contact_tabular().default_model(0.05, 1e9)
default_element = scene.contact_tabular().default_element()

# create constituiton
abd = AffineBodyConstitution()
scene.constitution_tabular().insert(abd)

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
sgui = SceneGUI(scene)

ps.init()
ps.set_ground_plane_height(0.0)
sgui.register()

run = False
def on_update():
    global run
    if(psim.Button("run & stop")):
        run = not run

    if(run):
        world.advance()
        world.retrieve()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()