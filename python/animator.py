import numpy as np
import polyscope as ps
import polyscope.imgui as psim
from pyuipc_loader import pyuipc
from pyuipc_loader import AssetDir
from pyuipc import Matrix4x4, Logger
from pyuipc.world import *
from pyuipc.engine import *
from pyuipc.constitution import *
from pyuipc.geometry import *

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

Logger.set_level(Logger.Level.Info)

workspace = AssetDir.output_path(__file__)

engine = Engine("cuda", workspace)
world = World(engine)

config = Scene.default_config()
config['contact']['d_hat'] = 0.005
print(config)

scene = Scene(config)

snk = StableNeoHookean()
scene.constitution_tabular().insert(snk)
scene.contact_tabular().default_model(0.5, 1e9)
default_element = scene.contact_tabular().default_element()

Vs = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
Ts = np.array([[0,1,2,3]])
tet = tetmesh(Vs, Ts)
process_surface(tet)

moduli = ElasticModuli.youngs_poisson(1e4, 0.49)
snk.apply_to(tet, moduli)
default_element.apply_to(tet)

pos_v = view(tet.positions())
for i in range(len(pos_v)):
    pos_v[i][1] += 0.2

object = scene.objects().create("object")
object.geometries().create(tet)

animator = world.animator()

def animation(info:Animation.UpdateInfo):
    geos:list[SimplicialComplex] = info.geo_slots()
    print(geos[0].meta().to_json())
    pass

animator.insert(object, animation)

sio = SceneIO(scene)
world.init(scene)
sio.write_surface(f'{workspace}/scene_surface0.obj')

run = False
ps.init()
ps.set_ground_plane_mode('none')
s = sio.simplicial_surface()
v = s.positions().view()
t = s.triangles().topo().view()
mesh = ps.register_surface_mesh('obj', v.reshape(-1,3), t.reshape(-1,3))
mesh.set_edge_width(1.0)
def on_update():
    global run
    if(psim.Button("run & stop")):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        s = sio.simplicial_surface()
        v = s.positions().view()
        mesh.update_vertex_positions(v.reshape(-1,3))

ps.set_user_callback(on_update)
ps.show()

