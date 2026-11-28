import numpy as np
import pytest

from uipc import geometry
from uipc.geometry import affine_body


def as_layout(array, layout):
    if layout == "fortran":
        return np.asfortranarray(array)
    if layout == "strided":
        shape = (2 * len(array), *array.shape[1:-1], 2 * array.shape[-1])
        padded = np.zeros(shape, dtype=array.dtype)
        sliced = padded[::2, ..., ::-2]
        sliced[...] = array
        return sliced
    if layout == "readonly":
        array.flags.writeable = False
    return array


@pytest.mark.parametrize("layout", ["c", "fortran", "strided", "readonly"])
@pytest.mark.parametrize("factory, elements, facets", [
    (geometry.pointcloud, None, None),
    (geometry.linemesh, [[1, 0], [3, 2]], "edges"),
    (geometry.trimesh, [[1, 2, 0], [0, 2, 3]], "triangles"),
    (geometry.trimesh, [[0, 1, 2, 3], [4, 3, 2, 1]], "triangles"),
    (geometry.tetmesh, [[0, 1, 2, 3], [4, 3, 2, 1]], "tetrahedra"),
], ids=["points", "edges", "triangles", "quads", "tetrahedra"])
def test_mesh_factory_array_layout(factory, elements, facets, layout):
    vertices = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]])
    positions = as_layout(vertices, layout)
    args = [positions]
    if elements is not None:
        indices = np.array(elements, dtype=np.int32)
        topology = as_layout(indices, layout)
        if layout in ("fortran", "strided"):
            topology.flags.writeable = False
        args.append(topology)
    if layout == "strided":
        args = [array[..., None] for array in args]  # Native column-vector form (N,K,1).

    mesh = factory(*args)
    np.testing.assert_array_equal(mesh.positions().view()[..., 0], vertices)
    if facets is not None:
        expected = indices
        if factory is geometry.trimesh and indices.shape[1] == 4:
            expected = indices[:, [0, 1, 2, 0, 2, 3]].reshape(-1, 3)
        actual = getattr(mesh, facets)().topo().view()
        np.testing.assert_array_equal(actual[..., 0], expected)


@pytest.mark.parametrize("layout", ["c", "fortran", "strided", "readonly"])
@pytest.mark.parametrize("forward, inverse, last_entry", [
    (affine_body.q_to_transform, affine_body.transform_to_q, 1),
    (affine_body.q_v_to_transform_v, affine_body.transform_v_to_q_v, 0),
], ids=["pose", "velocity"])
def test_abd_batch_matches_scalar(forward, inverse, last_entry, layout):
    vectors = np.array([[3., 5, 7, 0, -1, 0, 1, 0, 0, 0, 0, 1],
                        [-2., 4, 6, 1, 0, 0, 0, 0, -1, 0, 1, 0]])
    matrices = np.zeros((2, 4, 4))
    matrices[:, :3, 3] = vectors[:, :3]
    matrices[:, :3, :3] = vectors[:, 3:].reshape(2, 3, 3)
    matrices[:, 3, 3] = last_entry

    batch_matrices = forward(as_layout(vectors, layout))
    batch_vectors = inverse(as_layout(matrices, layout))
    np.testing.assert_array_equal(batch_matrices, matrices)
    np.testing.assert_array_equal(batch_vectors, vectors[..., None])
    for i in range(len(vectors)):
        np.testing.assert_array_equal(batch_matrices[i], forward(vectors[i]))
        np.testing.assert_array_equal(batch_vectors[i], inverse(matrices[i]))


def test_empty_geometry_array_batches():
    assert geometry.pointcloud(np.empty((0, 3, 1))).vertices().size() == 0
    for forward, inverse in ((affine_body.q_to_transform, affine_body.transform_to_q),
                             (affine_body.q_v_to_transform_v, affine_body.transform_v_to_q_v)):
        assert forward(np.empty((0, 12))).shape == (0, 4, 4)
        assert inverse(np.empty((0, 4, 4))).shape == (0, 12, 1)
