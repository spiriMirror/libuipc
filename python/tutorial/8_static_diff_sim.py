from pyuipc_loader import pyuipc
from pyuipc import Vector3, Logger

from pyuipc.geometry import *
from pyuipc import view

from pyuipc.core import World, Scene, SceneIO, Engine
from pyuipc.constitution import HookeanSpring
from pyuipc import builtin
from asset import AssetDir
import numpy as np

Logger.set_level(Logger.Level.Warn)
output_path = AssetDir.output_path(__file__)

engine = Engine('cuda', output_path)
world = World(engine)

config = Scene.default_config()
config['dt'] = 1.0
config['contact']['enable'] = False
config['diff_sim']['enable'] = True
scene = Scene(config)

diff_sim = scene.diff_sim()
diff_sim.parameters().resize(1, 0.0)

hs = HookeanSpring()
obj = scene.objects().create('rods')

n = 2
Vs = np.array([[0, 0, 0], [0, -0.1, 0]], dtype=np.float32)
Es = np.array([[0, 1]], dtype=np.int32)

mesh = linemesh(Vs, Es)
label_surface(mesh)
hs.apply_to(mesh, 1e8)
mesh.edges().create('diff/kappa', np.int32(0))
is_fixed = mesh.vertices().find(builtin.is_fixed)
is_fixed_view = view(is_fixed)
is_fixed_view[0] = 1

geo_slot, rest_geo_slot = obj.geometries().create(mesh)
print(mesh.edges().to_json())
world.init(scene)

sio = SceneIO(scene)
world.dump()

Frame = 1

aim_y = -0.4
error = 0.001

def YofX(X:np.ndarray) -> float:
    return X[-2] 

losK = 1e5

def Loss(X:np.ndarray, aim_Y:float) -> float:
    Y = YofX(X)
    return 0.5 * losK * (Y - aim_Y) ** 2

def dLossdY(Y:float, aim_Y:float) -> float:
    return losK * (Y - aim_Y)

def dLossdX(X:np.ndarray, aim_Y:float) -> np.ndarray:
    Y = YofX(X)
    ret = np.zeros_like(X)
    y = YofX(X)
    ret[-2] = dLossdY(y, aim_Y)
    return ret

parm_view = view(diff_sim.parameters())
parm_view[:] = 100.0

Xs = None

def forward_and_backward():
    while(world.frame() < Frame):
        world.advance()
        world.backward()
        world.retrieve()
        sio.write_surface(f"{output_path}/scene_surface.{world.frame()}.obj")
        
    pos_view = geo_slot.geometry().positions().view()
    Xs = pos_view.reshape(-1).copy()
    return Xs

def forward_only():
    while(world.frame() < Frame):
        world.advance()
        world.retrieve()
        sio.write_surface(f"{output_path}/scene_surface.{world.frame()}.obj")
    
    pos_view = geo_slot.geometry().positions().view()
    Xs = pos_view.reshape(-1).copy()
    return Xs

for epoch in range(100):
    # clear the stored hessian and gradient
    diff_sim.clear()
    # broadcast the parameters to the simulator
    diff_sim.parameters().broadcast()
    
    # recover at frame 0
    world.recover(0)
    world.retrieve()
    sio.write_surface(f"{output_path}/scene_surface.{epoch}.{world.frame()}.obj")
    
    Xs = forward_and_backward()
    
    parm_view = view(diff_sim.parameters())
    current_parm_view = parm_view.copy()
    
    loss = Loss(Xs, aim_y)
    print(f'epoch: {epoch}, loss: {loss}')
    
    pGpP = diff_sim.pGpP().to_dense()
    H = diff_sim.H().to_dense()
    dXdP = np.linalg.solve(H, -pGpP)
    
    dLdX = dLossdX(Xs, aim_y)
    dLdP = np.dot(dLdX, dXdP)
    Y = YofX(Xs)
    
    sio.write_surface(f"{output_path}/scene_surface.epoch.{epoch}.obj")
    
    if(np.abs(Y - aim_y) < error):
        print(f'Converged at epoch {epoch}')
        break
    
    alpha = 1.0
    
    # gradient descent
    while(True):
        parm_view[:] -= alpha * dLdP
        diff_sim.parameters().broadcast()
        print('kappa:', parm_view)
        
        world.recover(0)
        Xs = forward_only()
        
        new_loss = Loss(Xs, aim_y)
        print(f'epoch: {epoch}, loss: {new_loss} / {loss}, alpha: {alpha}')
        
        if(new_loss <= loss):
            loss = new_loss
            break
        else:
            alpha *= 0.5