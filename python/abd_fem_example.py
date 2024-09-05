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
print(config)


scene = Scene(config)

snk = StableNeoHookean()
abd = AffineBodyConstitution()
scene.constitution_tabular().insert(snk)
scene.constitution_tabular().insert(abd)
scene.contact_tabular().default_model(0.5, 1e9)
default_element = scene.contact_tabular().default_element()

pre_trans = pyuipc.Matrix4x4.Identity()

io = SimplicialComplexIO(pre_trans)
cube = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
cube = process_surface(cube)

fem_cube = cube.copy()
moduli = ElasticModuli.youngs_poisson(1e5, 0.49)
snk.apply_to(fem_cube, moduli)
default_element.apply_to(fem_cube)

abd_cube = cube.copy()
abd.apply_to(abd_cube, 1e8)
default_element.apply_to(abd_cube)

object = scene.objects().create("object")
N = 15

trans = Matrix4x4.Identity()

for i in range(N):
    geo = None
    if i % 2 == 0:
        geo = fem_cube.copy()
        pos_v = view(geo.positions())
        for j in range(len(pos_v)):
            pos_v[j][1] += 1.2 * i
    else:
        geo = abd_cube.copy()
        pos_v = view(geo.positions())
        for j in range(len(pos_v)):
            pos_v[j][1] += 1.2 * i
    object.geometries().create(geo)

g = ground(-1.2)
object.geometries().create(g)

sio = SceneIO(scene)
world.init(scene)

run = False

ps.init()
ps.set_ground_plane_mode('none')
s = sio.simplicial_surface()

ssio = SpreadSheetIO(workspace)
ssio.write_csv('surf', s)

v = s.positions().view()
t = s.triangles().topo().view()
mesh = ps.register_surface_mesh('obj', v.reshape(-1,3), t.reshape(-1,3))
mesh.set_edge_width(1.0)
def on_update():
    global run
    if(psim.Button('run & stop')):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        s = sio.simplicial_surface()
        v = s.positions().view()
        mesh.update_vertex_positions(v.reshape(-1,3))

ps.set_user_callback(on_update)
ps.show()
