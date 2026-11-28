import gc
import weakref

import numpy as np
import pytest

from uipc import Scene, SceneIO, view
from uipc.backend import SceneVisitor
from uipc.geometry import pointcloud


@pytest.mark.parametrize("get", [
    lambda s: s.config(), lambda s: s.objects(), lambda s: s.geometries(),
    lambda s: s.contact_tabular().contact_models(),
    lambda s: s.subscene_tabular().subscene_models(),
    SceneIO,
    lambda s: SceneVisitor(s).geometries(),
])
def test_scene_facade_retains_owner(get):
    scene = Scene()
    owner = weakref.ref(scene)
    facade = get(scene)
    del scene
    gc.collect()
    assert owner() is not None
    del facade
    gc.collect()
    assert owner() is None


@pytest.mark.parametrize("lookup", [lambda scene: scene.objects().find(0),
                                    lambda scene: scene.objects().find("body")[0]])
def test_object_lookup_keeps_scene_for_geometry_access(lookup):
    scene = Scene()
    scene.objects().create("body")
    obj = lookup(scene)
    temporary_collections = []
    for _ in range(8):
        objects = scene.objects()
        temporary_collections.append(weakref.ref(objects))
        assert objects.find(obj.id()) is obj
        assert objects.find("body")[0] is obj
    del objects
    assert all(ref() is None for ref in temporary_collections)
    del scene
    gc.collect()
    geometry = obj.geometries()
    owner = weakref.ref(obj)
    del obj
    gc.collect()
    assert owner() is not None
    geometry.create(pointcloud(np.zeros((1, 3))))
    assert geometry.ids().size == 1
    del geometry
    gc.collect()
    assert owner() is None


@pytest.mark.parametrize("method", ["meta", "instances", "vertices", "edges", "triangles", "tetrahedra"])
def test_geometry_facade_retains_owner(method):
    mesh = pointcloud(np.zeros((1, 3)))
    owner = weakref.ref(mesh)
    facade = getattr(mesh, method)()
    del mesh
    gc.collect()
    assert owner() is not None
    del facade
    gc.collect()
    assert owner() is None


@pytest.mark.parametrize("get_span", [lambda slot: slot.view(), view])
def test_string_span_retains_attribute_slot(get_span):
    mesh = pointcloud(np.zeros((1, 3)))
    slot = mesh.vertices().create("label", "v0")
    owner = weakref.ref(slot)
    span = get_span(slot)
    del slot, mesh
    gc.collect()
    assert owner() is not None
    assert span[0] == "v0"
    del span
    gc.collect()
    assert owner() is None
