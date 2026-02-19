import pytest 
import uipc as uipc 
import numpy as np
import polyscope as ps
import uipc
from uipc import Logger
from uipc import Engine, World, Scene, SceneIO
from uipc import Matrix4x4
from uipc.geometry import SimplicialComplex, SimplicialComplexIO
from uipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from uipc.geometry import ground, tetmesh, merge, extract_surface
from uipc.constitution import StableNeoHookean, ElasticModuli
from asset import AssetDir
from uipc import view

@pytest.mark.basic
def test_transform():
    Transform = uipc.Transform
    AngleAxis = uipc.AngleAxis
    Vector3 = uipc.Vector3

    # create a tetrahedron
    Vs = np.array([[0,1,0],
                [0,0,1],
                [-np.sqrt(3)/2, 0, -0.5],
                [np.sqrt(3)/2, 0, -0.5]])

    Ts = np.array([[0,1,2,3]])

    T = Transform.Identity()
    A = AngleAxis(np.pi/4, Vector3.UnitZ())
    T.rotate(A)

    assert Vs.shape == (4, 3), "single tet has 4 vertices"
    sc1 = tetmesh(Vs, Ts)
    pos1_view = view(sc1.positions())
    T.apply_to(pos1_view)
    # rotate by pi/4 around Z: first vertex (0,1,0) should not equal original
    assert not np.allclose(pos1_view.reshape(-1, 3), Vs), "apply_to(rotate) changes positions"

    T = Transform.Identity()
    T.translate(Vector3.UnitY() * 2)
    T.scale(1.2)
    sc2 = tetmesh(Vs, Ts)
    pos2_view = view(sc2.positions())
    T.apply_to(pos2_view)
    # translated and scaled: y component should be 2 + 1.2*original
    assert np.allclose(pos2_view.reshape(-1, 3)[:, 1], 2.0 + 1.2 * Vs[:, 1]), "apply_to(translate+scale) correct"

    sc = merge([sc1, sc2])
    assert sc.vertices().size() == 8, "merge two tets gives 8 vertices"
    assert sc.tetrahedra().size() == 2, "merge two tets gives 2 tetrahedra"
    label_surface(sc)
    surf = extract_surface(sc)
    assert surf is not None and surf.triangles().size() > 0, "extract_surface yields surface with triangles"

