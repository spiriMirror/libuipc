import numpy as np
from pyuipc_loader import pyuipc

world = pyuipc.world
geometry = pyuipc.geometry

scene = world.Scene()
obj = scene.objects().create("obj")

ground = geometry.ground()
geo, rest_geo = obj.geometries().create(ground)

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