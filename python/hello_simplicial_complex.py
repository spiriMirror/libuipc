import numpy as np
from pyuipc_loader import pyuipc

uipc = pyuipc
geometry = uipc.geometry

Vs = np.array([
    [1,0,0],
    [0,1,0],
    [0,0,1],
    [0,0,0]])

Ts = np.array([[0,1,2,3]])

sc = geometry.tetmesh(Vs, Ts)

print("positions", sc.positions().view())

# define velocity attribute
vel = sc.vertices().create("velocity", uipc.Vector3.Zero())
vel_view:np.ndarray[np.float64] = geometry.view(vel)

for i in range(vel_view.shape[0]):
    vel_view[i,:,:] = uipc.Vector3.Ones() * i

print("vel:\n", vel_view)
sc.vertices().destroy("velocity")
find_vel = sc.vertices().find("velocity")
print("find_vel:\n", find_vel)

# get topology
print("edges:\n", sc.edges().topo().view())
print("triangles:\n", sc.triangles().topo().view())
print("tetrahedra:\n", sc.tetrahedra().topo().view())

try:
    I64 = sc.tetrahedra().create("i64", np.zeros((10, 10), np.int64))
except RuntimeError as e:
    print(e)




