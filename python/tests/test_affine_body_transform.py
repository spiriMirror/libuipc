"""CPU regressions for the batched affine-body conversions and as_span_of's array layout.

The batched q_to_transform, transform_to_q, q_v_to_transform_v and transform_v_to_q_v
must agree with their single-item forms for every memory layout of the input, including
uipc's own (column-major) matrix views. as_span_of views a buffer in place, so it must
refuse an array that is not C-contiguous instead of reading other values.
"""

import numpy as np
import pytest

from uipc import Exception as UIPCException
from uipc import view
from uipc.geometry import affine_body, pointcloud


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
def test_as_span_of_refuses_non_contiguous_arrays():
    wide = np.arange(24, dtype=np.float64).reshape(4, 6)
    positions = wide[:, :3]  # four rows of three, six values apart
    assert not positions.flags.c_contiguous
    with pytest.raises(UIPCException, match="C-contiguous"):
        pointcloud(positions)
    sc = pointcloud(np.ascontiguousarray(positions))
    np.testing.assert_array_equal(view(sc.positions()).reshape(-1, 3), positions)
