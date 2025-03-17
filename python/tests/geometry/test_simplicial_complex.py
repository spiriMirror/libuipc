import pytest 

import numpy as np
import uipc as uipc 
from uipc import view
from uipc import geometry

@pytest.mark.basic
def test_simplicial():
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
    vel_view:np.ndarray[np.float64] = view(vel)

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

    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = geometry.tetmesh(Vs, Ts)

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

    mesh = geometry.tetmesh(Vs, Ts)
    mesh.vertices().create("velocity", np.zeros(3))

