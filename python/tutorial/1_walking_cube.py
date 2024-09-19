import numpy as np
import polyscope as ps
from polyscope import imgui

from asset import AssetDir
from pyuipc_loader import pyuipc
from pyuipc import Vector3, Vector2, Transform, Logger, Matrix4x4
from pyuipc import builtin

from pyuipc.engine import *
from pyuipc.world import *
from pyuipc.geometry import *
from pyuipc.constitution import *
from pyuipc_utils.gui import *


Logger.set_level(Logger.Level.Warn)
output_path = AssetDir.output_path(__file__)


engine = Engine('cuda', output_path)
world = World(engine)

config = Scene.default_config()
dt = 0.02
config['dt'] = dt
config['newton']['velocity_tol'] = 0.05
scene = Scene(config)

# friction ratio and contact resistance
scene.contact_tabular().default_model(0.2, 1e9)
default_element = scene.contact_tabular().default_element()

# create constituiton
abd = AffineBodyConstitution()
scene.constitution_tabular().insert(abd)

# create constraint
stc = SoftTransformConstraint()
scene.constitution_tabular().insert(stc)

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    return flip_inward_triangles(sc)

io = SimplicialComplexIO()
cube_mesh = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
cube_mesh = process_surface(cube_mesh)

# move the cube up for 2.5 meters
trans_view = view(cube_mesh.transforms())
t = Transform.Identity()
t.translate(Vector3.UnitY() * 2.5)
trans_view[0] = t.matrix()

abd.apply_to(cube_mesh, 1e8) # 100 MPa
default_element.apply_to(cube_mesh)
# constraint the rotation
stc.apply_to(cube_mesh, Vector2.Values([0, 100.0]))
cube_object = scene.objects().create('cube')
cube_object.geometries().create(cube_mesh)

pre_transform = Transform.Identity()
pre_transform.scale(Vector3.Values([3, 0.1, 6]))

io = SimplicialComplexIO(pre_transform)
ground_mesh = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
ground_mesh = process_surface(ground_mesh)
ground_mesh.instances().resize(2)

abd.apply_to(ground_mesh, 1e7) # 10 MPa
default_element.apply_to(ground_mesh)
stc.apply_to(ground_mesh, Vector2.Values([100.0, 100.0]))
is_fixed = ground_mesh.instances().find(builtin.is_fixed)
is_fixed_view = view(is_fixed)
is_fixed_view[0] = 1 # fix the lower board
is_fixed_view[1] = 0

trans_view = view(ground_mesh.transforms())
t = Transform.Identity()
t.translate(Vector3.UnitZ() * 2)
trans_view[0] = t.matrix()
t.translate(Vector3.UnitZ() * -2.5 + Vector3.UnitY() * 1)
trans_view[1] = t.matrix()

ground_object = scene.objects().create('ground')
ground_object.geometries().create(ground_mesh)

ground_height = -1.0
g = ground(ground_height)
ground_object.geometries().create(g)

animator = scene.animator()

def cube_animation(info:Animation.UpdateInfo):
    geo_slots = info.geo_slots()
    geo_slot: SimplicialComplexSlot = geo_slots[0]
    geo = geo_slot.geometry()
    is_constrained = geo.instances().find(builtin.is_constrained)
    view(is_constrained)[0] = 1
    
    trans = geo.instances().find(builtin.transform)
    aim = geo.instances().find(builtin.aim_transform)
    trans_view = trans.view()
    aim_view = view(aim)
    
    # Get the rotation matrix from the current transform
    R = trans_view[0][0:3,0:3]
    e_i = R[:,0]
    e_j = R[:,1]
    e_k = R[:,2]
    
    # Rotate the cube around the e_i axis
    angular_velocity = np.pi # 180 degree per second
    theta = angular_velocity * dt
    new_e_j = np.cos(theta) * e_j + np.sin(theta) * e_k
    new_e_k = -np.sin(theta) * e_j + np.cos(theta) * e_k
    
    R[:,0] = e_i
    R[:,1] = new_e_j
    R[:,2] = new_e_k
    
    aim_view[0][0:3,0:3] = R

def ground_animation(info:Animation.UpdateInfo):
    geo_slot: SimplicialComplexSlot = info.geo_slots()[0]
    rest_geo_slot : SimplicialComplexSlot = info.rest_geo_slots()[0]
    geo = geo_slot.geometry()
    rest_geo = rest_geo_slot.geometry()
    
    is_constrained = geo.instances().find(builtin.is_constrained)
    view(is_constrained)[1] = 1
    
    current_t = dt * info.frame()
    angular_velocity = np.pi # 180 degree per second
    theta = angular_velocity * current_t
    
    T:Matrix4x4 = rest_geo.transforms().view()[1]
    Y = np.sin(theta) * 0.4
    T:Transform = Transform(T)
    p = T.translation()
    p[1] += Y
    T = Transform.Identity()
    T.translate(p)
    
    aim_trans = geo.instances().find(builtin.aim_transform)
    view(aim_trans)[1] = T.matrix()

animator.insert(cube_object, cube_animation)
animator.insert(ground_object, ground_animation)

world.init(scene)
sgui = SceneGUI(scene)

ps.init()
ps.set_ground_plane_height(ground_height)
tri_surf, _, _ = sgui.register()
tri_surf.set_edge_width(1)

run = False
def on_update():
    global run
    if(imgui.Button('run & stop')):
        run = not run

    if(run):
        world.advance()
        world.retrieve()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()