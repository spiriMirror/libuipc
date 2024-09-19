import numpy as np
import polyscope as ps
from polyscope import imgui

from pyuipc_loader import pyuipc
from pyuipc import Vector3, Vector2, Transform, Logger, Quaternion, AngleAxis
from pyuipc import builtin

from pyuipc.engine import *
from pyuipc.world import *
from pyuipc.geometry import *
from pyuipc.constitution import *
from pyuipc_utils.gui import *
from asset import AssetDir

Logger.set_level(Logger.Level.Warn)

workspace = AssetDir.output_path(__file__)

engine = Engine("cuda", workspace)
world = World(engine)

config = Scene.default_config()
config['dt'] = 0.01
config['newton']['velocity_tol'] = 0.01
print(config)
scene = Scene(config)

hs = HookeanSpring()
krb = KirchhoffRodBending()
scene.constitution_tabular().insert(hs)
scene.constitution_tabular().insert(krb)
scene.contact_tabular().default_model(0.05, 1e9)
default_element = scene.contact_tabular().default_element()

n = 8
Vs = np.zeros((n, 3), dtype=np.float32)
for i in range(n):
    Vs[i][2] = i # Z
Vs *= 0.03
Vs += np.array([0, 0.1, 0.0]) # move up a bit
Es = np.zeros((n-1, 2), dtype=np.int32)
for i in range(n-1):
    Es[i] = [i, i+1]

object = scene.objects().create("rods")
rods = 6
for i in range(rods):
    Vs += np.array([0.04, 0, 0])
    mesh = linemesh(Vs, Es)
    label_surface(mesh)
    hs.apply_to(mesh, 40.0 * 1e6)
    krb.apply_to(mesh, i * 1e9)
    default_element.apply_to(mesh)
    
    is_fixed = mesh.vertices().find(builtin.is_fixed)
    is_fixed_view = view(is_fixed)
    # fix first 2 vertices
    is_fixed_view[0] = 1
    is_fixed_view[1] = 1
    object.geometries().create(mesh)

ground_height = -0.1
ground_obj = scene.objects().create("ground")
g = ground(ground_height)
ground_obj.geometries().create(g)

sio = SceneIO(scene)
sgui = SceneGUI(scene)

world.init(scene)

run = False

ps.init()
ps.set_ground_plane_height(ground_height)

_, mesh1d, _ =sgui.register()
mesh1d.set_radius(0.01, False)

def on_update():
    global run
    if(imgui.Button("run & stop")):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()
