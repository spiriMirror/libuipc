import pytest 
import numpy as np
import math 
import os 
import pathlib
import json 
import polyscope as ps
import polyscope.imgui as psim
from pyuipc_loader import pyuipc as uipc
from pyuipc_loader import \
    Engine, World, Scene, SceneIO \
    ,Object, ContactElement, Animation

from asset import AssetDir

from pyuipc_utils.geometry import \
    SimplicialComplex, SimplicialComplexIO \
    ,SimplicialComplexSlot \
    ,SpreadSheetIO \
    ,label_surface, label_triangle_orient, flip_inward_triangles\
    ,ground, view, linemesh

from pyuipc_utils.constitution import \
    StableNeoHookean, AffineBodyConstitution, ElasticModuli, \
    SoftPositionConstraint, HookeanSpring

from pyuipc_utils.gui import SceneGUI

def rotate_x(angle):
    A = angle[0]
    I = uipc.Matrix3x3.Identity()
    I[0,0] = np.cos(A)
    I[0,1] = -np.sin(A)
    I[1,0] = np.sin(A)
    I[1,1] = np.cos(A)
    return I

def rotate_y(angle):
    A = angle[0]
    I = uipc.Matrix3x3.Identity()
    I[0,0] = np.cos(A)
    I[0,2] = np.sin(A)
    I[2,0] = -np.sin(A)
    I[2,2] = np.cos(A)
    return I

def rotate_z(angle):
    A = angle[0]
    I = uipc.Matrix3x3.Identity()
    I[1,1] = np.cos(A)
    I[1,2] = -np.sin(A)
    I[2,1] = np.sin(A)
    I[2,2] = np.cos(A)
    return I

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

run = False

@pytest.mark.example 
def test_wrecking_balls():
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    workspace = AssetDir.output_path(__file__)
    folder = AssetDir.folder(__file__)

    engine = Engine("cuda", workspace)
    world = World(engine)

    config = Scene.default_config()
    config["contact"]["d_hat"]              = 0.01
    config["line_search"]["max_iter"]       = 8
    config["newton"]["velocity_tol"]       = 0.05
    print(config)

    scene = Scene(config)
    abd = AffineBodyConstitution()
    scene.constitution_tabular().insert(abd)
    scene.contact_tabular().default_model(0, 1e10)
    default_contact = scene.contact_tabular().default_element()

    pre_trans = uipc.Matrix4x4.Identity()
    io = SimplicialComplexIO(pre_trans)


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
        position = uipc.Vector3.Zero()
        if "position" in json:
            position[0] = json["position"][0]
            position[1] = json["position"][1]
            position[2] = json["position"][2]
        
        rot_matrix = uipc.Matrix3x3.Identity()
        if "rotation" in json:
            rotation = uipc.Vector3.Zero()
            rotation[0] = json["rotation"][0]
            rotation[1] = json["rotation"][1]
            rotation[2] = json["rotation"][2]
            rotation *= np.pi / 180
            
            rot_matrix = rotate_x(rotation[2]) @ rotate_y(rotation[1]) @ rotate_z(rotation[0])
            
        is_fixed = 0
        if "is_dof_fixed" in json:
            is_fixed = json["is_dof_fixed"]
        
        trans_matrix = uipc.Matrix4x4.Identity()
        trans_matrix[0:3, 0:3] = rot_matrix
        trans_matrix[0:3, 3] = position.reshape(3)
        
        this_mesh = mesh.copy()
        view(this_mesh.transforms())[0] = trans_matrix
        
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

    pre_trans = uipc.Matrix4x4.Identity()
    pre_trans[0,0] = 20
    pre_trans[1,1] = 0.2
    pre_trans[2,2] = 20

    io = SimplicialComplexIO(pre_trans)
    g = io.read(f'{tetmesh_dir}/cube.msh')
    process_surface(g)

    trans_matrix = uipc.Matrix4x4.Identity()
    trans_matrix[0:3, 3] = np.array([12, -1.1, 0])
    view(g.transforms())[0] = trans_matrix
    abd.apply_to(g, 1e7)

    is_fixed = g.instances().find("is_fixed")
    view(is_fixed)[0] = 1

    ground_obj = scene.objects().create("ground")
    ground_obj.geometries().create(g)

    sio = SceneIO(scene)
    world.init(scene)

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

