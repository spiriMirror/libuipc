from pyuipc_tutorial import * # common import for all tutorial scripts
# =================================================================================================
from pyuipc import Vector3, Vector2, Transform
from pyuipc.engine import Engine
from pyuipc.geometry import \
    tetmesh, label_surface, label_triangle_orient, flip_inward_triangles, view, ground,\
    SimplicialComplex, SimplicialComplexSlot, SimplicialComplexIO
from pyuipc.world import World, Scene, SceneIO, Animation
from pyuipc.constitution import AffineBodyConstitution, SoftTransformConstraint
from pyuipc import builtin
from pyuipc_utils.gui import SceneGUI
import polyscope as ps
import polyscope.imgui as psim
import numpy as np

output_path = AssetDir.output_path(__file__)

engine = Engine('cuda', output_path)
world = World(engine)
dt = 0.01
config = Scene.default_config()
config['dt'] = dt
scene = Scene()

# friction ratio and contact resistance
scene.contact_tabular().default_model(0.5, 1e9)
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
is_fixed = ground_mesh.instances().find(builtin.is_fixed)
view(is_fixed).fill(1)
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

def animation(info:Animation.UpdateInfo):
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

animator.insert(cube_object, animation)

world.init(scene)
sgui = SceneGUI(scene)

ps.init()
ps.set_ground_plane_height(ground_height)
sgui.register()

run = False
def on_update():
    global run
    if(psim.Button('run & stop')):
        run = not run

    if(run):
        world.advance()
        world.retrieve()
        sgui.update()

ps.set_user_callback(on_update)
ps.show()