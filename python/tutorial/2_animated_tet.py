import numpy as np
import polyscope as ps
from polyscope import imgui

from asset import AssetDir
from pyuipc_loader import pyuipc

from pyuipc import Vector3, Vector2, Transform, Logger, Timer
from pyuipc import builtin
from pyuipc.core import World, Scene, SceneIO, Engine, Animation

from pyuipc.geometry import *
from pyuipc.constitution import *
from pyuipc_utils.gui import *

from asset import AssetDir

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

run = False

Timer.enable_all()
Logger.set_level(Logger.Level.Info)

workspace = AssetDir.output_path(__file__)

engine = Engine("cuda", workspace)
world = World(engine)

config = Scene.default_config()
dt = 0.02
config['dt'] = dt
config['contact']['friction']['enable'] = False
print(config)

scene = Scene(config)

snh = StableNeoHookean()
spc = SoftPositionConstraint()
scene.contact_tabular().default_model(0.5, 1e9)
default_element = scene.contact_tabular().default_element()

N = 6
Vs = np.array([[0, 1, 0], 
            [0, 0, 1], 
            [-np.sqrt(3)/2, 0, -0.5], 
            [np.sqrt(3)/2, 0, -0.5]
            ], dtype=np.float32)
Ts = np.array([[0,1,2,3]])
base_tet = tetmesh(Vs, Ts)
process_surface(base_tet)
moduli = ElasticModuli.youngs_poisson(1e5, 0.49)
snh.apply_to(base_tet, moduli)
spc.apply_to(base_tet)
default_element.apply_to(base_tet)

object = scene.objects().create("object")
for i in range(N):
    t = Transform.Identity()
    s = 1 if i % 2 == 0 else 0
    t.translate(Vector3.Values([2 * i, s * 0.5, 0]))
    tet = base_tet.copy()
    pos_view = view(tet.positions())
    t.apply_to(pos_view)
    object.geometries().create(tet)

ground_height = -0.2
g = ground(ground_height)
scene.objects().create("ground").geometries().create(g)

# scripted animation
def animation(info:Animation.UpdateInfo):
    geo_slots:list[GeometrySlot] = info.geo_slots()
    rest_geo_slots:list[GeometrySlot] = info.rest_geo_slots()
    for i in range(N):
        geo_slot = geo_slots[i]
        rest_geo_slot = rest_geo_slots[i]
        geo:SimplicialComplex = geo_slot.geometry()
        rest_geo:SimplicialComplex = rest_geo_slot.geometry()
        # label the constrained vertices
        is_constrained = geo.vertices().find(builtin.is_constrained)
        is_constrained_view = view(is_constrained)
        is_constrained_view[0] = 1 # constrain the first vertex
        
        rest_pos = rest_geo.positions()
        rest_pos_view = view(rest_pos)
        P = rest_pos_view[0]

        t = info.frame() * dt
        # set the aim position
        apos = geo.vertices().find(builtin.aim_position)
        apos_view = view(apos)
        
        theta = np.pi * t
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        h = sin_t if i % 2 == 1 else cos_t - 1
        
        apos_view[0] = P + Vector3.Values([0, h * 0.5, 0])
    pass

scene.animator().substep(1)
scene.animator().insert(object, animation)

world.init(scene)

sio = SceneIO(scene)
sgui = SceneGUI(scene)

ps.init()
ps.set_ground_plane_height(ground_height)
tri_surf, _, _ = sgui.register()
tri_surf.set_edge_width(1)
def on_update():
    global run
    if(imgui.Button("run & stop")):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        Timer.report()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()

