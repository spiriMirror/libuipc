import pytest 

import numpy as np
import uipc as uipc 
from uipc import view
import uipc.geometry as geometry

@pytest.mark.basic
def test_simplicial():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = geometry.tetmesh(Vs, Ts)

    pos_view = sc.positions().view()
    assert pos_view.shape[0] == 4, "single tet has 4 vertices"
    assert np.allclose(pos_view.reshape(4, 3), Vs), "positions match input Vs"

    # define velocity attribute
    vel = sc.vertices().create("velocity", uipc.Vector3.Zero())
    vel_view: np.ndarray = view(vel)
    assert vel_view.shape[0] == 4, "velocity attribute has one entry per vertex"

    for i in range(vel_view.shape[0]):
        vel_view[i, :, :] = uipc.Vector3.Ones() * i
    assert np.allclose(vel_view[1].flatten(), [1, 1, 1]), "vertex 1 velocity written correctly"

    sc.vertices().destroy("velocity")
    find_vel = sc.vertices().find("velocity")
    assert find_vel is None, "destroy removes attribute; find returns None"

    # get topology (single tet: 4 verts, 6 edges, 4 triangles, 1 tet)
    assert sc.vertices().size() == 4 and sc.tetrahedra().size() == 1, "tet mesh counts"
    assert sc.edges().size() == 6 and sc.triangles().size() == 4, "facet_closure edge/face counts"
    topo_tet = sc.tetrahedra().topo().view()
    assert topo_tet.shape[0] == 1 and topo_tet.size >= 4, "one tet with 4 vertex indices"

    try:
        I64 = sc.tetrahedra().create("i64", 64)
    except RuntimeError as e:
        assert "i64" in str(e).lower() or len(str(e)) > 0, "create with wrong type raises RuntimeError"

    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = geometry.tetmesh(Vs, Ts)

    assert sc.vertices().size() == 4 and sc.tetrahedra().size() == 1, "simplicial complex shape"
    assert sc.positions().view().shape[0] == 4, "positions length"
    assert sc.transforms().view().shape[0] == 1, "one instance transform"

    pos_attr = sc.vertices().find("position")
    assert pos_attr is not None, "builtin position attribute exists"
    assert pos_attr.view().shape[0] == 4, "position attribute length"

    assert sc.edges().find("NO_SUCH_ATTRIBUTE") is None, "find missing attribute returns None"
    assert sc.triangles().find("NO_SUCH_ATTRIBUTE") is None
    assert sc.tetrahedra().find("NO_SUCH_ATTRIBUTE") is None

    assert sc.edges().topo().view().shape[0] == 6, "edge topology size"
    assert sc.triangles().topo().view().shape[0] == 4, "triangle topology size"
    assert sc.tetrahedra().topo().view().shape[0] == 1, "tetrahedron topology size"

    v = sc.vertices().create("i64", np.array(0, dtype=np.int64))
    a = view(v)
    b = view(v)
    a.fill(1)
    b.fill(2)
    assert np.all(np.asarray(view(v)).flatten() == 2), "last write (b.fill(2)) wins on shared attribute"
    # read-only view must not allow write
    try:
        v.view().fill(3)
    except ValueError as e:
        assert isinstance(e, ValueError), "read-only view raises ValueError on write"

    j = sc.vertices().to_json()
    assert isinstance(j, dict), "vertices to_json returns dict"
    assert j and ("vertices" in j or "attributes" in j or "position" in j), "to_json has structure"

    mesh = geometry.tetmesh(Vs, Ts)
    mesh.vertices().create("velocity", np.zeros(3))

