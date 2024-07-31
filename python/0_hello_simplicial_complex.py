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
vel = sc.vertices().create("velocity", np.zeros((3, 1), np.float64))
vel_view:np.ndarray[np.float64] = geometry.view(vel)

for i in range(vel_view.shape[0]):
    vel_view[i,:,:] = np.ones((3, 1), np.float64) * i

print("vel:\n", vel_view)
sc.vertices().destroy("velocity")
find_vel = sc.vertices().find("velocity")
print("find_vel:\n", find_vel)

# get topology
print("edges:\n", sc.edges().topo().view())
print("triangles:\n", sc.triangles().topo().view())
print("tetrahedra:\n", sc.tetrahedra().topo().view())





