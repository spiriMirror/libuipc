from . import uipc 

__all__ = [
    "tetmesh",
    "trimesh",
    "pointcloud",
    "linemesh",
    "ground",
    "label_surface",
    "label_triangle_orient",
    "flip_inward_triangles",
    "GeometrySlot",
    "SimplicialComplex",
    "SimplicialComplexIO"
]

geom = uipc["geometry"]
tetmesh = geom.tetmesh
trimesh = geom.trimesh
pointcloud = geom.pointcloud
linemesh = geom.linemesh
ground = geom.ground
label_surface = geom.label_surface
label_triangle_orient = geom.label_triangle_orient
flip_inward_triangles = geom.flip_inward_triangles
GeometrySlot = geom.GeometrySlot
SimplicialComplex = geom.SimplicialComplex
SimplicialComplexIO = geom.SimplicialComplexIO