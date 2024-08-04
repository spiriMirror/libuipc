import numpy as np
import polyscope as ps
import polyscope.imgui as psim
from pyuipc_loader import pyuipc
from pyuipc_loader import AssetDir

geometry = pyuipc.geometry
constitution = pyuipc.constitution
workspace = AssetDir.output_path(__file__)

engine = pyuipc.engine.Engine("cuda", workspace)
world = pyuipc.world.World(engine)
scene = pyuipc.world.Scene()

abd = constitution.AffineBodyConstitution()
scene.constitution_tabular().create(abd)
default_element = scene.contact_tabular().default_element()

io = geometry.SimplicialComplexIO()
cube = io.read(f'{AssetDir.tetmesh_path()}/cube.msh')
geometry.label_surface(cube)
geometry.label_triangle_orient(cube)
cube = geometry.flip_inward_triangles(cube)

abd.apply_to(cube, 1e8)
default_element.apply_to(cube)

object = scene.objects().create("object")

N = 10
trans = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], np.float64)
for i in range(N):
    trans[0:3, 3] = np.array([0, 1.1 * (i + 1), 0])
    geometry.view(cube.transforms())[0] = trans
    object.geometries().create(cube)

ground = geometry.ground(0.0)
object.geometries().create(ground)

sio = pyuipc.world.SceneIO(scene)
world.init(scene)

run = False
use_gui = True
if use_gui:
    ps.init()
    ps.set_ground_plane_height_factor(0.5, False)
    s = sio.surface()
    v = s.positions().view()
    t = s.triangles().topo().view()
    mesh = ps.register_surface_mesh('obj', v.reshape(-1,3), t.reshape(-1,3))
    
    def on_update():
        global run
        if(psim.Button("run&stop")):
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
    while(world.frame() < 10):
        world.advance()
        world.retrieve()
        sio.write_surface(f'{workspace}/scene_surface{world.frame()}.obj')
        world.dump()






    

    
