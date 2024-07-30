from pyuipc_loader import pyuipc
import numpy as np

geometry = pyuipc.geometry
SimplicialComplex = geometry.SimplicialComplex
tetmesh = geometry.tetmesh

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

pos = sc.vertices().find("position").view()

print(pos)

print(sc.edges().find("NO_SUCH_ATTRIBUTE"))

print(sc.triangles().find("NO_SUCH_ATTRIBUTE"))

print(sc.tetrahedra().find("NO_SUCH_ATTRIBUTE"))

print(sc.vertices().topo())

print(sc.edges().topo())

print(sc.triangles().topo())

print(sc.tetrahedra().topo())



