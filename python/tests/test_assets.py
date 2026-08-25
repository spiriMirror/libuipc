import numpy as np
import pytest

from uipc import Scene
from uipc.assets import strip_constitutions
from uipc.geometry import pointcloud


@pytest.mark.basic
def test_strip_constitutions_handles_sparse_object_ids():
    scene = Scene()
    first = scene.objects().create("first")
    second = scene.objects().create("second")
    survivor = scene.objects().create("survivor")

    mesh = pointcloud(np.zeros((1, 3), dtype=np.float64))
    mesh.meta().create("constitution_uid", np.uint64(10))
    geometry, _ = survivor.geometries().create(mesh)

    scene.objects().destroy(first.id())
    scene.objects().destroy(second.id())
    assert scene.objects().size() == 1
    assert scene.objects().created_count() == 3
    assert geometry.geometry().meta().find("constitution_uid") is not None

    strip_constitutions(scene)

    assert geometry.geometry().meta().find("constitution_uid") is None
