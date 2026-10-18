"""CPU regressions for issue #498: stitch index ownership and array layout."""

import numpy as np
import pytest

from uipc import Scene, builtin
from uipc import Exception as UIPCException
from uipc.constitution import SoftVertexStitch
from uipc.geometry import pointcloud


VERTEX_COUNT = 8194


@pytest.fixture
def stitch_scene():
    scene = Scene()
    obj = scene.objects().create("stitch")
    vertices = np.zeros((VERTEX_COUNT, 3), dtype=np.float64)
    left, _ = obj.geometries().create(pointcloud(vertices))
    vertices[:, 2] = 1.0
    right, _ = obj.geometries().create(pointcloud(vertices))
    contacts = scene.contact_tabular()
    elements = (contacts.create("left"), contacts.create("right"))
    # Keep the owning scene alive while its geometry slots are used.
    yield (left, right), elements


def make_pairs(count, dtype):
    ids = np.arange(count, dtype=dtype)
    return np.column_stack((2 * ids, 2 * ids[::-1] + 1))


def create_stitch(stitch_scene, pairs, with_contacts, kappa=1e4, rest_length=0.25):
    slots, elements = stitch_scene
    kwargs = dict(kappa=kappa, rest_length=rest_length)
    if with_contacts:
        kwargs["contact_elements"] = elements
    return SoftVertexStitch().create_geometry(slots, pairs, **kwargs)


def assert_stitch(stitch_scene, geo, expected, with_contacts, kappa=1e4):
    topo = np.asarray(geo.instances().find(builtin.topo).view()).reshape(-1, 2)
    np.testing.assert_array_equal(topo, expected)
    np.testing.assert_array_equal(
        geo.instances().find("kappa").view().reshape(-1),
        np.full(len(expected), kappa),
    )
    np.testing.assert_array_equal(
        geo.instances().find("rest_length").view().reshape(-1),
        np.full(len(expected), 0.25),
    )
    if with_contacts:
        slots, elements = stitch_scene
        for column, (slot, element) in enumerate(zip(slots, elements)):
            expected_ids = np.zeros(VERTEX_COUNT, dtype=np.int32)
            expected_ids[expected[:, column]] = element.id()
            actual = slot.geometry().vertices().find(builtin.contact_element_id)
            np.testing.assert_array_equal(actual.view().reshape(-1), expected_ids)


@pytest.mark.parametrize("with_contacts", [False, True], ids=["plain", "contacts"])
@pytest.mark.parametrize("dtype", [np.int32, np.int64], ids=["int32", "int64"])
@pytest.mark.parametrize("count", [0, 1, 16, 64, 128, 256, 300, 512, 1024, 4096])
def test_stitch_preserves_indices(stitch_scene, with_contacts, dtype, count):
    pairs = make_pairs(count, dtype)
    # Repeated allocations expose the allocator-dependent freed-buffer bug.
    # Check both columns independently and retain prior geometries across calls.
    geometries = []
    for attempt in range(5):
        kappa = 1e4 + attempt
        geo = create_stitch(stitch_scene, pairs, with_contacts, kappa=kappa)
        assert_stitch(stitch_scene, geo, pairs, with_contacts, kappa=kappa)
        geometries.append(geo)
    expected = pairs.copy()
    pairs.fill(0)
    for attempt, geo in enumerate(geometries):
        assert_stitch(stitch_scene, geo, expected, with_contacts, kappa=1e4 + attempt)


@pytest.mark.parametrize("with_contacts", [False, True], ids=["plain", "contacts"])
@pytest.mark.parametrize("dtype", [np.int32, np.int64], ids=["int32", "int64"])
@pytest.mark.parametrize(
    "layout",
    [
        "fortran", "row_slice", "column_slice", "reverse_rows",
        "reverse_columns", "transpose", "readonly", "vector_shape",
    ],
)
def test_stitch_array_layout(stitch_scene, with_contacts, dtype, layout):
    pairs = make_pairs(300, dtype)
    if layout == "fortran":
        pairs = np.asfortranarray(pairs)
    elif layout == "row_slice":
        pairs = make_pairs(600, dtype)[::2]
    elif layout == "column_slice":
        padded = np.full((300, 4), -1, dtype=dtype)
        padded[:, ::2] = pairs
        pairs = padded[:, ::2]
    elif layout == "reverse_rows":
        pairs = pairs[::-1]
    elif layout == "reverse_columns":
        pairs = pairs[:, ::-1]
    elif layout == "transpose":
        pairs = np.ascontiguousarray(pairs.T).T
    elif layout == "readonly":
        pairs.flags.writeable = False
    elif layout == "vector_shape":
        pairs = pairs.reshape(-1, 2, 1)
    expected = pairs.reshape(-1, 2).copy()
    geo = create_stitch(stitch_scene, pairs, with_contacts)
    np.testing.assert_array_equal(pairs.reshape(-1, 2), expected)
    del pairs
    assert_stitch(stitch_scene, geo, expected, with_contacts)


@pytest.mark.parametrize("with_contacts", [False, True], ids=["plain", "contacts"])
@pytest.mark.parametrize("shape", [(4,), (4, 3), (4, 2, 2), (4, 2, 1, 1)])
def test_stitch_rejects_invalid_shape(stitch_scene, with_contacts, shape):
    with pytest.raises(UIPCException, match="Shape mismatch|array must be"):
        create_stitch(stitch_scene, np.zeros(shape, dtype=np.int32), with_contacts)
