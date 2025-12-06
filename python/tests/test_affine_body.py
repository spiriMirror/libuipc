import pytest 
import numpy as np
import polyscope as ps
import polyscope.imgui as psim
from uipc import Logger
from uipc import Matrix4x4
from uipc import Engine, World, Scene, SceneIO
from uipc.geometry import SimplicialComplex, SimplicialComplexIO
from uipc.geometry import SpreadSheetIO
from uipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from uipc.geometry import ground, view
from uipc.constitution import AffineBodyConstitution
from asset import AssetDir

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    label_triangle_orient(sc)
    sc = flip_inward_triangles(sc)
    return sc

run = False 

@pytest.mark.example
def test_affine_body():
    Logger.set_level(Logger.Level.Info)
    workspace = AssetDir.output_path(__file__)
    engine = Engine("cuda", workspace)
    world = World(engine)
    config = Scene.default_config()
    print(config)
    scene = Scene(config)

    abd = AffineBodyConstitution()
    
    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    pre_trans = Matrix4x4.Identity()

    # scaling
    pre_trans[0,0] = 0.2
    pre_trans[1,1] = 0.2
    pre_trans[2,2] = 0.2

    io = SimplicialComplexIO(pre_trans)
    cube = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
    cube = process_surface(cube)

    abd.apply_to(cube, 1e8)
    default_element.apply_to(cube)

    object = scene.objects().create("object")

    N = 30


    trans = Matrix4x4.Identity()

    for i in range(N):
        trans[0:3, 3] = np.array([0, 0.24 * (i + 1), 0])
        view(cube.transforms())[0] = trans
        object.geometries().create(cube)

    g = ground(0.0)
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
