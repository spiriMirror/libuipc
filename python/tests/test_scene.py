import numpy as np
import pytest

from uipc import Scene
from uipc.geometry import ground, tetmesh

@pytest.mark.basic 
def test_scene():
    scene = Scene()
    Vs = np.array([[0, 1, 0], 
                [0, 0, 1], 
                [-np.sqrt(3)/2, 0, -0.5], 
                [np.sqrt(3)/2, 0, -0.5]
                ], dtype=np.float32)
    Ts = np.array([[0,1,2,3]])
    tet = tetmesh(Vs, Ts)

    obj = scene.objects().create("obj")

    g = ground()
    geo, rest_geo = obj.geometries().create(g)

    obj.geometries().create(tet)

    geo_json = geo.geometry().to_json()
    rest_json = rest_geo.geometry().to_json()
    assert isinstance(geo_json, dict) and isinstance(rest_json, dict), "geometry to_json returns dict"

    find_geo, find_rest_geo = scene.geometries().find(geo.id())
    assert find_geo.id() == find_rest_geo.id(), "find by id returns same geo and rest_geo id"
    assert find_geo.id() == geo.id(), "found geometry id matches created geometry id"

    ids = obj.geometries().ids()
    assert len(ids) == 2, "object has two geometries (ground + tet)"
    assert geo.id() in ids, "created ground geometry id is in object's geometry ids"

    found_obj = scene.objects().find(obj.id())
    assert found_obj is not None, "object exists before destroy"
    scene.objects().destroy(obj.id())
    assert scene.objects().find(obj.id()) is None, "object is gone after destroy"