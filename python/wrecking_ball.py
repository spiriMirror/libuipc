import numpy as np
import polyscope as ps
import json
import polyscope.imgui as psim
from pyuipc_loader import pyuipc
from pyuipc_loader import AssetDir

geometry = pyuipc.geometry
constitution = pyuipc.constitution

def rotate_x(angle):
    A = angle[0]
    I = pyuipc.Matrix3x3.Identity()
    I[0,0] = np.cos(A)
    I[0,1] = -np.sin(A)
    I[1,0] = np.sin(A)
    I[1,1] = np.cos(A)
    return I

def rotate_y(angle):
    A = angle[0]
    I = pyuipc.Matrix3x3.Identity()
    I[0,0] = np.cos(A)
    I[0,2] = np.sin(A)
    I[2,0] = -np.sin(A)
    I[2,2] = np.cos(A)
    return I

def rotate_z(angle):
    A = angle[0]
    I = pyuipc.Matrix3x3.Identity()
    I[1,1] = np.cos(A)
    I[1,2] = -np.sin(A)
    I[2,1] = np.sin(A)
    I[2,2] = np.cos(A)
    return I

def process_surface(sc: pyuipc.geometry.SimplicialComplex):
    geometry.label_surface(sc)
    geometry.label_triangle_orient(sc)
    sc = geometry.flip_inward_triangles(sc)
    return sc

pyuipc.no_debug_info()
workspace = AssetDir.output_path(__file__)
folder = AssetDir.folder(__file__)

engine = pyuipc.engine.Engine("cuda", workspace)
world = pyuipc.world.World(engine)

config = pyuipc.world.Scene.default_config()
config["contact"]["d_hat"]              = 0.01
config["line_search"]["max_iter"]       = 8
print(config)

scene = pyuipc.world.Scene(config)
abd = constitution.AffineBodyConstitution()
scene.constitution_tabular().create(abd)
scene.contact_tabular().default_model(0.5, 1e10)
default_contact = scene.contact_tabular().default_element()

pre_trans = pyuipc.Matrix4x4.Identity()
io = geometry.SimplicialComplexIO(pre_trans)


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

def build_mesh(json, obj:pyuipc.world.Object, mesh:geometry.SimplicialComplex):
    position = pyuipc.Vector3.Zero()
    if "position" in json:
        position[0] = json["position"][0]
        position[1] = json["position"][1]
        position[2] = json["position"][2]
    
    rot_matrix = pyuipc.Matrix3x3.Identity()
    if "rotation" in json:
        rotation = pyuipc.Vector3.Zero()
        rotation[0] = json["rotation"][0]
        rotation[1] = json["rotation"][1]
        rotation[2] = json["rotation"][2]
        rotation *= np.pi / 180
        
        rot_matrix = rotate_x(rotation[2]) @ rotate_y(rotation[1]) @ rotate_z(rotation[0])
        
    is_fixed = 0
    if "is_dof_fixed" in json:
        is_fixed = json["is_dof_fixed"]
    
    trans_matrix = pyuipc.Matrix4x4.Identity()
    trans_matrix[0:3, 0:3] = rot_matrix
    trans_matrix[0:3, 3] = position.reshape(3)
    
    this_mesh = mesh.copy()
    geometry.view(this_mesh.transforms())[0] = trans_matrix
    
    is_fixed_attr = this_mesh.instances().find("is_fixed")
    geometry.view(is_fixed_attr)[0] = is_fixed
    
    obj.geometries().create(this_mesh)

for obj in wrecking_ball_scene:
    if obj['mesh'] == 'link.msh':
        build_mesh(obj, link_obj, link)
    elif obj['mesh'] == 'ball.msh':
        build_mesh(obj, ball_obj, ball)
    elif obj['mesh'] == 'cube.msh':
        build_mesh(obj, cube_obj, cube)

pre_trans = pyuipc.Matrix4x4.Identity()
pre_trans[0,0] = 20
pre_trans[1,1] = 0.2
pre_trans[2,2] = 20

io = geometry.SimplicialComplexIO(pre_trans)
ground = io.read(f'{tetmesh_dir}/cube.msh')
geometry.label_surface(ground)
geometry.label_triangle_orient(ground)

trans_matrix = pyuipc.Matrix4x4.Identity()
trans_matrix[0:3, 3] = np.array([12, -1.1, 0])
geometry.view(ground.transforms())[0] = trans_matrix
abd.apply_to(ground, 1e7)

is_fixed = ground.instances().find("is_fixed")
geometry.view(is_fixed)[0] = 1

ground_obj = scene.objects().create("ground")
ground_obj.geometries().create(ground)

sio = pyuipc.world.SceneIO(scene)
world.init(scene)

run = False
use_gui = True
if use_gui:
    ps.init()
    ps.set_ground_plane_mode('none')
    
    s = sio.surface()
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
            s = sio.surface()
            v = s.positions().view()
            mesh.update_vertex_positions(v.reshape(-1,3))
    
    ps.set_user_callback(on_update)
    ps.show()
else:
    # try recover from the previous state
    sio.write_surface(f'{workspace}/scene_surface{0}.obj')
    world.recover()
    while(world.frame() < 1000):
        world.advance()
        world.retrieve()
        sio.write_surface(f'{workspace}/scene_surface{world.frame()}.obj')
        world.dump()


