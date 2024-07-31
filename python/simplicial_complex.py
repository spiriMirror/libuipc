import numpy as np
from pyuipc_loader import pyuipc
from pyuipc.geometry import *

view = pyuipc.geometry.view

Vs = np.array([
    [1,0,0],
    [0,1,0],
    [0,0,1],
    [0,0,0]])

Ts = np.array([[0,1,2,3]])

sc = tetmesh(Vs, Ts)

print(sc)

print(sc.meta())

print(sc.instances())

print(sc.vertices())

print(sc.edges())

print(sc.triangles())

print(sc.tetrahedra())

print(sc.positions())

print(sc.positions().view())

print(sc.transforms().view())

print(sc.vertices().find("position"))

print(sc.vertices().find("position").view())

print(sc.edges().find("NO_SUCH_ATTRIBUTE"))

print(sc.triangles().find("NO_SUCH_ATTRIBUTE"))

print(sc.tetrahedra().find("NO_SUCH_ATTRIBUTE"))

print(sc.vertices().topo())

print(sc.edges().topo())

print(sc.triangles().topo())

print(sc.tetrahedra().topo())

print(sc.vertices().topo().view())

print(sc.edges().topo().view())

print(sc.triangles().topo().view())

print(sc.tetrahedra().topo().view())

print(type(np.uint64(0)), np.uint64(0))

v = sc.vertices().create("i64", np.array(0, dtype=np.int64))

#view(v)[0] = 1
a = view(v)
b = view(v)

a.fill(1)
b.fill(2)

del b
del a

print(view(v))

topo = sc.vertices().topo()
topo_a = view(topo)
topo_b = view(topo)

del topo

topo_a.fill(1)
topo_b.fill(4)

del topo_b
del topo_a

print(view(sc.vertices().topo()))






