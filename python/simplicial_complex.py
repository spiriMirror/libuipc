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

print("API test:")
print("object:")
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

print("view:")
print(sc.vertices().topo().view())
print(sc.edges().topo().view())
print(sc.triangles().topo().view())
print(sc.tetrahedra().topo().view())

print("attribute create")
v = sc.vertices().create("i64", np.array(0, dtype=np.int64))
print(v)
a = view(v)
b = view(v)
# allow write to the read-write view
a.fill(1)
b.fill(2)
# not allow write to the read-only view
try:
    v.view().fill(3)
except ValueError as e:
    print('caught ValueError:', e)

print(v.view())

print("attribute destroy:")
j = sc.vertices().to_json()
print(j)



mesh = tetmesh(Vs, Ts)

mesh.vertices().create("velocity", np.zeros(3))






