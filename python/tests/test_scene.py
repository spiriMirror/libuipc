import numpy as np
from pyuipc_loader import pyuipc
from pyuipc import Logger
from pyuipc import Engine, World, Scene, SceneIO
from pyuipc import Matrix4x4
from pyuipc.geometry import SimplicialComplex, SimplicialComplexIO
from pyuipc.geometry import label_surface, label_triangle_orient, flip_inward_triangles
from pyuipc.geometry import ground, view, tetmesh
from pyuipc.constitution import StableNeoHookean, ElasticModuli
from asset import AssetDir


import pytest 

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