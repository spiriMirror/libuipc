import numpy as np
from pyuipc_loader import pyuipc


import pytest 

@pytest.mark.basic 
def test_scene():
    world = pyuipc.world
    geometry = pyuipc.geometry
    scene = world.Scene()
    Vs = np.array([[0, 1, 0], 
                [0, 0, 1], 
                [-np.sqrt(3)/2, 0, -0.5], 
                [np.sqrt(3)/2, 0, -0.5]
                ], dtype=np.float32)
    Ts = np.array([[0,1,2,3]])
    tet = geometry.tetmesh(Vs, Ts)


    obj = scene.objects().create("obj")

    ground = geometry.ground()
    geo, rest_geo = obj.geometries().create(ground)

    obj.geometries().create(tet)

    print(geo.geometry().to_json())
    print(rest_geo.geometry().to_json())

    find_geo, find_rest_geo = scene.geometries().find(geo.id())
    print(find_geo.id())
    print(find_rest_geo.id())
    assert find_geo.id() == find_rest_geo.id()

    print(obj.geometries().ids())

    print(scene.objects().find(obj.id()))
    scene.objects().destroy(obj.id())
    print(scene.objects().find(obj.id()))