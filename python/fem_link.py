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

pre_trans = pyuipc.Matrix4x4.Identity()

# scaling
pre_trans[0,0] = 0.2
pre_trans[1,1] = 0.2
pre_trans[2,2] = 0.2

io = SimplicialComplexIO(pre_trans)
link = io.read(f'{AssetDir.tetmesh_path()}/link.msh')
link = process_surface(link)

moduli = ElasticModuli.youngs_poisson(1e4, 0.49)
snk.apply_to(link, moduli)
default_element.apply_to(link)

pos_v = view(link.positions())
for i in range(len(pos_v)):
    pos_v[i][1] += 0.2

object = scene.objects().create("object")
object.geometries().create(link)

g = ground(0.0)
object.geometries().create(g)

sio = SceneIO(scene)
world.init(scene)
sio.write_surface(f'{workspace}/scene_surface0.obj')

while(world.frame() < 200):
    world.advance()
    world.retrieve()
    sio.write_surface(f'{workspace}/scene_surface{world.frame()}.obj')
