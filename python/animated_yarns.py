import numpy as np
import polyscope as ps
import polyscope.imgui as psim
import os
import pathlib
import math
from pyuipc_loader import pyuipc
from pyuipc_loader import AssetDir
from pyuipc import Vector3, Matrix4x4, Logger
from pyuipc import builtin
from pyuipc.world import *
from pyuipc.engine import *
from pyuipc.constitution import *
from pyuipc.geometry import *

def process_surface(sc: SimplicialComplex):
    label_surface(sc)
    return sc

class Yarns:
    def __init__(self, path) -> None:
        self.positions = []
        f = open(path, 'r')
        for line in f:
            x, y, z = line.split(',')
            self.positions.append(Vector3.Values([float(x), float(y), float(z)]))
        f.close()
        self.positions = np.array(self.positions, dtype=np.float32)
        self.yarn_time_step = self.positions[1][1] - self.positions[0][1]
        print('yarn time step:', self.yarn_time_step)
        self.current_id = 0
        self.last_t = 0.0
    
    def create_geometry(self, object: Object, 
                        spc:SoftPositionConstraint, 
                        hs:HookeanSpring,
                        contact_element:ContactElement,
                        segment_height, count, thickness):
        start_point = self.positions[0]
        Vs = []
        for i in range(count):
            Vs.append(start_point + Vector3.Values([0, i * segment_height, 0]))
        Vs = np.array(Vs, dtype=np.float32)
        
        Es = []
        for i in range(count - 1):
            Es.append([i, i + 1])
        Es = np.array(Es, dtype=np.int32)
        mesh = linemesh(Vs, Es)
        process_surface(mesh)
        spc.apply_to(mesh)
        contact_element.apply_to(mesh)
        hs.apply_to(mesh, 1e9, 1e3, thickness)
        
        is_constrained = mesh.vertices().find(builtin.is_constrained)
        is_constrained_view = view(is_constrained)
        is_constrained_view[0] = 1 # constrain the first point
        is_constrained_view[-1] = 1 # constrain the last point
        
        object.geometries().create(mesh)
    
    def _round_index(self, i):
        return i % len(self.positions)
    
    def next_position(self, dt):
        aim = dt + self.last_t
        d_index = int(math.floor(aim / self.yarn_time_step))
        self.current_id = self._round_index(self.current_id + d_index)
        next_id = self._round_index(self.current_id + 1)
        next_pos = self.positions[next_id]
        cur_pos = self.positions[self.current_id]
        cur_pos[1] = 0.0 # clear the time component
        self.last_t = aim - d_index * self.yarn_time_step
        tmp = float(self.last_t / self.yarn_time_step)
        v = next_pos - cur_pos
        v[1] = 0.0 # clear the time component
        dx = v * tmp
        pos = cur_pos + dx
        return pos
    
    def move(self, geo:SimplicialComplex, dt:float, move_up_speed = 0.2):
        pos = self.next_position(dt)
        aim_pos = geo.vertices().find(builtin.aim_position)
        aim_pos_view = view(aim_pos)
        aim_pos_view[0] = pos
        aim_pos_view[-1][1]+= dt * move_up_speed

class Braider:
    def __init__(self, folder) -> None:
        # find all file starting with 'yarn_' and ending with '.txt'
        self.files: list[str] = []
        for file in os.listdir(folder):
            if file.startswith('yarn_') and file.endswith('.txt'):
                abs_path = os.path.join(folder, file)
                abs_path = pathlib.Path(abs_path).resolve().absolute()
                self.files.append(str(abs_path))
        print("yarn files:", self.files)
        self.yarns: list[Yarns] = []
        for file in self.files:
            self.yarns.append(Yarns(file))
        self.object: Object = None
    
    def create_geometries(self, scene: Scene, segment_len = 0.1, count=30, thickness=0.02):
        self.object = scene.objects().create("yarns")
        spc = SoftPositionConstraint()
        hs = HookeanSpring()
        scene.constitution_tabular().insert(spc)
        scene.constitution_tabular().insert(hs)
        ce = scene.contact_tabular().default_element()
        for yarn in self.yarns:
            yarn.create_geometry(self.object, spc, hs, ce, segment_len, count, thickness)
    
    def create_animation(self, scene: Scene, dt = 0.01, move_up_speed = 0.2):
        def anim(info:Animation.UpdateInfo):
            geo_slots:list[SimplicialComplexSlot] = info.geo_slots()
            for slot, yarns in zip(geo_slots, self.yarns):
                geo = slot.geometry()
                yarns.move(geo, dt, move_up_speed)
            pass
        scene.animator().insert(self.object, anim)
        

Logger.set_level(Logger.Level.Info)

workspace = AssetDir.output_path(__file__)

engine = Engine("cuda", workspace)
world = World(engine)

config = Scene.default_config()
config['gravity'] = Vector3.Values([0, 0, 0]).tolist()
config['contact']['friction']['enable'] = False
print(config)

scene = Scene(config)

scene.contact_tabular().default_model(0.5, 1e9)

thickness = 0.05
braider = Braider("yarns")
braider.create_geometries(scene, segment_len=0.2, count=20, thickness=thickness)
braider.create_animation(scene, move_up_speed=0)

world.init(scene)

sio = SceneIO(scene)
sio.write_surface(f'{workspace}/scene_surface{0}.obj')

run = False
ps.init()
ps.set_ground_plane_height(-1.2)
s = sio.simplicial_surface()
v = s.positions().view()
e = s.edges().topo().view()
mesh = ps.register_curve_network('obj', v.reshape(-1,3), e.reshape(-1,2))
mesh.set_radius(thickness * 0.2)
particle = ps.register_point_cloud('particle', v.reshape(-1,3))
particle.set_radius(thickness * 0.26)
def on_update():
    global run
    if(psim.Button("run & stop")):
        run = not run
        
    if(run):
        world.advance()
        world.retrieve()
        s = sio.simplicial_surface()
        v = s.positions().view()
        mesh.update_node_positions(v.reshape(-1,3))
        particle.update_point_positions(v.reshape(-1,3))
        sio.write_surface(f'{workspace}/scene_surface{world.frame()}.obj')


ps.set_user_callback(on_update)
ps.show()