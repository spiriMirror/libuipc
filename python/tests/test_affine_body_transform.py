"""CPU regressions for the batched affine-body conversions and as_span_of's array layout.

The batched q_to_transform, transform_to_q, q_v_to_transform_v and transform_v_to_q_v
must agree with their single-item forms for every memory layout of the input, including
uipc's own (column-major) matrix views. as_span_of views a buffer in place, so it must
refuse an array that is not C-contiguous instead of reading other values.
"""

import numpy as np
import pytest

from uipc import Exception as UIPCException
from uipc import Scene, view
from uipc.constitution import AffineBodySphericalJoint
from uipc.geometry import affine_body, optimal_transform, pointcloud


LAYOUTS = ["c_order", "fortran", "row_slice", "column_slice", "reverse_rows", "transpose", "readonly"]


def arranged(a, layout):
    """The values of ``a`` (N, ...) stored in the given memory layout."""
    if layout == "c_order":
        return np.ascontiguousarray(a)
    if layout == "fortran":
        return np.asfortranarray(a)
    if layout == "row_slice":
        return np.repeat(a, 2, axis=0)[::2]
    if layout == "column_slice":
        padded = np.zeros(a.shape[:-1] + (2 * a.shape[-1],))
        padded[..., ::2] = a
        return padded[..., ::2]
    if layout == "reverse_rows":
        return a[::-1].copy()[::-1]
    if layout == "transpose":
        return np.ascontiguousarray(np.swapaxes(a, -1, -2)).swapaxes(-1, -2)
    if layout == "readonly":
        b = a.copy()
        b.flags.writeable = False
        return b
    raise ValueError(layout)


def random_q(n, seed):
    return np.random.default_rng(seed).standard_normal((n, 12))


@pytest.mark.basic
@pytest.mark.parametrize("layout", LAYOUTS)
@pytest.mark.parametrize("name", ["q_to_transform", "q_v_to_transform_v"])
def test_vectors_to_matrices_batch_matches_single(name, layout):
    convert = getattr(affine_body, name)
    q = random_q(5, 0)
    expected = np.stack([convert(qi) for qi in q])
    batch = convert(arranged(q, layout))
    assert batch.shape == (5, 4, 4)
    np.testing.assert_array_equal(batch, expected)


@pytest.mark.basic
@pytest.mark.parametrize("layout", LAYOUTS)
@pytest.mark.parametrize("name", ["transform_to_q", "transform_v_to_q_v"])
def test_matrices_to_vectors_batch_matches_single(name, layout):
    convert = getattr(affine_body, name)
    m = np.random.default_rng(1).standard_normal((5, 4, 4))
    expected = np.stack([convert(mi) for mi in m])
    batch = convert(arranged(m, layout))
    assert batch.shape == (5, 12, 1)
    np.testing.assert_array_equal(batch, expected)


@pytest.mark.basic
def test_batched_round_trip():
    q = random_q(5, 2)
    back = affine_body.transform_to_q(affine_body.q_to_transform(q))
    np.testing.assert_allclose(back.reshape(-1, 12), q, rtol=0, atol=1e-12)


@pytest.mark.basic
def test_transform_to_q_reads_uipc_matrix_views():
    # uipc stores Matrix4x4 column-major: its view is not C-contiguous.
    sc = pointcloud(np.zeros((1, 3)))
    sc.instances().resize(3)
    trans = view(sc.transforms())
    trans[:] = np.stack([affine_body.q_to_transform(qi) for qi in random_q(3, 3)])
    expected = np.stack([affine_body.transform_to_q(t) for t in trans])
    np.testing.assert_array_equal(affine_body.transform_to_q(trans), expected)


@pytest.mark.basic
def test_pointcloud_reads_non_contiguous_rows():
    wide = np.arange(24, dtype=np.float64).reshape(4, 6)
    positions = wide[:, :3]  # four rows of three, six values apart
    assert not positions.flags.c_contiguous
    sc = pointcloud(positions)
    np.testing.assert_array_equal(sc.positions().view().reshape(-1, 3), positions)


@pytest.mark.basic
def test_optimal_transform_accepts_readonly_and_checks_packed_vector_layout():
    points = np.vstack((np.zeros(3), np.eye(3)))
    points.flags.writeable = False
    np.testing.assert_allclose(optimal_transform(points, points), np.eye(4), atol=1e-12)

    with pytest.raises(UIPCException, match="C-contiguous"):
        optimal_transform(arranged(points, "row_slice"), points)
    unaligned = np.ndarray(points.shape, dtype=points.dtype,
                           buffer=bytearray(points.nbytes + 1), offset=1)
    unaligned[:] = points
    assert unaligned.flags.c_contiguous and not unaligned.flags.aligned
    with pytest.raises(UIPCException, match="align"):
        optimal_transform(points, unaligned)


@pytest.mark.basic
def test_spherical_joint_packed_inputs():
    scene = Scene()
    obj = scene.objects().create("two bodies")
    mesh = pointcloud(np.zeros((1, 3)))
    mesh.instances().resize(2)
    left, _ = obj.geometries().create(mesh)
    right, _ = obj.geometries().create(mesh)
    anchors = np.arange(6.0).reshape(2, 3)
    ids = np.array([0, 1], dtype=np.int32)
    strengths = np.array([1.0, 3.0])
    anchors.flags.writeable = False
    joint = AffineBodySphericalJoint()
    geometry = joint.create_geometry(anchors, [left, left], ids, [right, right], ids, strengths)
    np.testing.assert_array_equal(geometry.positions().view().reshape(-1, 3), anchors)
    np.testing.assert_array_equal(geometry.vertices().find("l_inst_id").view(), ids)
    np.testing.assert_array_equal(geometry.vertices().find("strength_ratio").view(), strengths)

    with pytest.raises(UIPCException, match="C-contiguous"):
        joint.create_geometry(anchors, [left, left], np.repeat(ids, 2)[::2],
                              [right, right], ids, strengths)
    unaligned = np.ndarray(ids.shape, dtype=ids.dtype, buffer=bytearray(ids.nbytes + 1), offset=1)
    unaligned[:] = ids
    with pytest.raises(UIPCException, match="align"):
        joint.create_geometry(anchors, [left, left], unaligned, [right, right], ids, strengths)
