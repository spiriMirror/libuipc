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


@pytest.mark.parametrize("layout", ["row_slice", "reverse", "fortran", "column_vector", "unaligned"])
def test_apply_to_updates_only_selected_vectors(layout):
    base = np.arange(36.0).reshape(6, 6)
    if layout == "fortran":
        base = np.asfortranarray(base)
    if layout == "unaligned":
        storage = bytearray(base.nbytes + 1)
        unaligned = np.ndarray(base.shape, dtype=base.dtype, buffer=storage, offset=1)
        unaligned[:] = base
        base = unaligned
    rows = slice(None, None, -2 if layout == "reverse" else 2)
    cols = slice(None, None, -2 if layout == "reverse" else 2)
    selected = base[rows, cols]
    transform = uipc.Transform.Identity()
    transform.translate(np.array([1., 2., 3.]))
    transform.rotate(uipc.AngleAxis(np.pi / 2, uipc.Vector3.UnitZ()))
    matrix = transform.matrix()
    expected = base.copy()
    expected[rows, cols] = selected @ matrix[:3, :3].T + matrix[:3, 3]
    if layout == "column_vector":
        selected = selected[..., None]
    assert transform.apply_to(selected) is selected
    np.testing.assert_allclose(base, expected, atol=1e-12)


def test_apply_to_validates_storage_before_writing():
    transform = uipc.Transform.Identity()
    transform.translate(np.ones(3))
    base = np.zeros(6)
    overlap = np.lib.stride_tricks.sliding_window_view(base, 3, writeable=True)
    with pytest.raises(ValueError, match="non-overlapping"):
        transform.apply_to(overlap)
    np.testing.assert_array_equal(base, np.zeros(6))
    readonly = base.reshape(2, 3)
    readonly.flags.writeable = False
    with pytest.raises(ValueError, match="writable"):
        transform.apply_to(readonly)
    with pytest.raises(TypeError, match="float64"):
        transform.apply_to(np.zeros((2, 3), dtype=np.float32))
    empty = np.empty((0, 3))
    assert transform.apply_to(empty) is empty
    np.testing.assert_array_equal(transform.apply_to([1, 2, 3]).ravel(), [2, 3, 4])

