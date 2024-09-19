import json
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

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

Logger.set_level(Logger.Level.Warn)
workspace = AssetDir.output_path(__file__)
folder = AssetDir.folder(__file__)

engine = Engine("cuda", workspace)
world = World(engine)

config = Scene.default_config()
config["dt"] = 0.033
config["contact"]["d_hat"]              = 0.01
config["line_search"]["max_iter"]       = 8
config["newton"]["velocity_tol"]       = 0.2
print(config)

scene = Scene(config)
abd = AffineBodyConstitution()
scene.constitution_tabular().insert(abd)
scene.contact_tabular().default_model(0.02, 1e10)
default_contact = scene.contact_tabular().default_element()

io = SimplicialComplexIO()

f = open(f'{folder}/wrecking_ball.json')
wrecking_ball_scene = json.load(f)

tetmesh_dir = AssetDir.tetmesh_path()

cube = io.read(f'{tetmesh_dir}/cube.msh')
cube = process_surface(cube)
ball = io.read(f'{tetmesh_dir}/ball.msh')
ball = process_surface(ball)
link = io.read(f'{tetmesh_dir}/link.msh')
link = process_surface(link)

cube_obj = scene.objects().create("cubes")
ball_obj = scene.objects().create("balls")
link_obj = scene.objects().create("links")

abd.apply_to(cube, 1e7)
default_contact.apply_to(cube)

abd.apply_to(ball, 1e7)
default_contact.apply_to(ball)

abd.apply_to(link, 1e7)
default_contact.apply_to(link)

def build_mesh(json, obj:Object, mesh:SimplicialComplex):
    t = Transform.Identity()
    position = Vector3.Zero()
    if "position" in json:
        position[0] = json["position"][0]
        position[1] = json["position"][1]
        position[2] = json["position"][2]
        t.translate(position)
    
    Q = Quaternion.Identity()
    if "rotation" in json:
        rotation = Vector3.Zero()
        rotation[0] = json["rotation"][0]
        rotation[1] = json["rotation"][1]
        rotation[2] = json["rotation"][2]
        rotation *= np.pi / 180
        Q = AngleAxis(rotation[2][0], Vector3.UnitZ())  * AngleAxis(rotation[1][0], Vector3.UnitY()) * AngleAxis(rotation[0][0], Vector3.UnitX())
        t.rotate(Q)
        
    is_fixed = 0
    if "is_dof_fixed" in json:
        is_fixed = json["is_dof_fixed"]
    
    this_mesh = mesh.copy()
    view(this_mesh.transforms())[0] = t.matrix()
    
    is_fixed_attr = this_mesh.instances().find("is_fixed")
    view(is_fixed_attr)[0] = is_fixed
    
    obj.geometries().create(this_mesh)

for obj in wrecking_ball_scene:
    if obj['mesh'] == 'link.msh':
        build_mesh(obj, link_obj, link)
    elif obj['mesh'] == 'ball.msh':
        build_mesh(obj, ball_obj, ball)
    elif obj['mesh'] == 'cube.msh':
        build_mesh(obj, cube_obj, cube)

ground_height = -1.0
g = ground(-1.0)
ground_obj = scene.objects().create("ground")
ground_obj.geometries().create(g)

sgui = SceneGUI(scene)
world.init(scene)
world.recover()
world.retrieve()

ps.init()
ps.set_ground_plane_height(ground_height)
tri_surf, _, _ = sgui.register()
tri_surf.set_edge_width(1)

run = False
def on_update():
    global run
    if(imgui.Button("run & stop")):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        world.dump()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()

