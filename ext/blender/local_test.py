import pytest 

from cent.blender.wrapper import blender_executive
from cent.blender.scene import simple_cube
import bpy 
import numpy as np 

import scipy 
import bmesh 

import pytetwild 

import polyscope as ps
import polyscope.imgui as psim
from pyuipc_loader import pyuipc as uipc
from pyuipc_loader import Engine, World, Scene, SceneIO, Object

# from asset import AssetDir

from pyuipc_utils.geometry import \
    SimplicialComplex, SimplicialComplexIO \
    ,SpreadSheetIO \
    ,label_surface, label_triangle_orient, flip_inward_triangles \
    ,ground, view, tetmesh

from pyuipc_utils.constitution import \
    StableNeoHookean, AffineBodyConstitution, ElasticModuli, \
    SoftPositionConstraint

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

run = False 

class TriMesh:
    def __init__(self, vertices, faces):
        self.vertices = vertices
        self.faces = faces

@blender_executive 
def phys_box(rootdir):
    cube1, camera, light = simple_cube()
    cube2 = bpy.data.objects.new("cube2", cube1.data)
    bpy.context.collection.objects.link(cube2)
    # move to (0, 0, 2)
    cube2.location = (0, 0, 4)

    objs = [cube1, cube2]
    tri_meshs = []
    for obj in objs:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        # print vertices
        verts = []
        for v in bm.verts:
            vert = np.array([v.co.x, v.co.y, v.co.z])
            verts.append(vert)
        
        faces = []
        # print faces
        for f in bm.faces:
            face = np.array([])
            for v in f.verts:
                face = np.append(face, v.index)
            faces.append(face)

        tri_meshs.append(TriMesh(np.array(verts), np.array(faces)))

    tets = []
    for tri_mesh in tri_meshs:
        print("Triangle Mesh: ")
        print(tri_mesh.vertices)
        print(tri_mesh.faces)
        v_out, tetra = pytetwild.tetrahedralize(tri_mesh.vertices, tri_mesh.faces, optimize=False, edge_length_fac=0.5)
        tet = tetmesh(v_out, tetra)
        test = process_surface(tet)
        tets.append(test)

    uipc.Logger.set_level(uipc.Logger.Level.Info)
    workspace = "D:/temp/"
    engine = Engine("cuda", workspace)
    world = World(engine)
    config = Scene.default_config()
    print(config)
    scene = Scene(config)
    abd = AffineBodyConstitution()
    scene.constitution_tabular().insert(abd)
    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    for tet in tets:
        abd.apply_to(tet, 1e8)
        default_element.apply_to(tet)
    
    object = scene.objects().create("object")
    N = 30
    trans = uipc.Matrix4x4.Identity()

    for tet in tets:
        slot, rest_slot = object.geometries().create(tet)
    
    g = ground(-0.6)
    object.geometries().create(g)

    sio = SceneIO(scene)
    world.init(scene)

    use_gui = True
    if use_gui:
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

                # print(slots[0].geometry().transforms().view())
        
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


phys_box("phys_box")