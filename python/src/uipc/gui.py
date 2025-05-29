import polyscope as ps
from . import Scene, SceneIO
from uipc import builtin
from uipc import core
from uipc.backend import SceneVisitor
from uipc.geometry import SimplicialComplexSlot, SimplicialComplex, extract_surface, apply_transform, merge
from typing import Literal
import numpy as np

# only export SceneGUI
__all__ = ['SceneGUI']

class _SceneGUIMerge:
    def __init__(self, scene:Scene):
        self.scene = scene
        self.scene_io = SceneIO(scene)
        self.trimesh:ps.SurfaceMesh = None
        self.linemesh:ps.CurveNetwork = None
        self.pointcloud:ps.PointCloud = None
    
    def register(self)->tuple[ps.SurfaceMesh, ps.CurveNetwork, ps.PointCloud]:
        trimesh = self.scene_io.simplicial_surface(2)
        linemesh = self.scene_io.simplicial_surface(1)
        pointcloud = self.scene_io.simplicial_surface(0)
        
        if(trimesh.vertices().size() != 0):
            self.trimesh = ps.register_surface_mesh('trimesh', trimesh.positions().view().reshape(-1,3), trimesh.triangles().topo().view().reshape(-1,3))
        if(linemesh.vertices().size() != 0):
            self.linemesh = ps.register_curve_network('linemesh', linemesh.positions().view().reshape(-1,3), linemesh.edges().topo().view().reshape(-1,2))
            thickness = linemesh.vertices().find(builtin.thickness)
            if thickness is not None:
                self.linemesh.set_radius(thickness.view()[0], relative=False)
        if(pointcloud.vertices().size() != 0):
            self.pointcloud = ps.register_point_cloud('pointcloud', pointcloud.positions().view().reshape(-1,3))
            thickness = pointcloud.vertices().find(builtin.thickness)
            if thickness is not None:
                thickness = thickness.view()
                self.pointcloud.add_scalar_quantity('thickness', thickness)
                self.pointcloud.set_point_radius_quantity('thickness', False)
        
        return self.trimesh, self.linemesh, self.pointcloud
    
    def update(self):
        if self.trimesh is not None:
            trimesh = self.scene_io.simplicial_surface(2)
            self.trimesh.update_vertex_positions(trimesh.positions().view().reshape(-1,3))
        if self.linemesh is not None:
            linemesh = self.scene_io.simplicial_surface(1)
            self.linemesh.update_node_positions(linemesh.positions().view().reshape(-1,3))
        if self.pointcloud is not None:
            pointcloud = self.scene_io.simplicial_surface(0)
            self.pointcloud.update_point_positions(pointcloud.positions().view().reshape(-1,3))
    
    def set_edge_width(self, width:float):
        if self.trimesh is not None:
            self.trimesh.set_edge_width(width)

class _SceneGUISplit:
    def __init__(self, scene:Scene):
        self.scene_visitor = SceneVisitor(scene)
        self.tetmeshes:  dict[int,tuple[SimplicialComplexSlot,ps.VolumeMesh]] = {}
        self.trimeshes:  dict[int,tuple[SimplicialComplexSlot,ps.SurfaceMesh]] = {}
        self.linemeshes: dict[int,tuple[SimplicialComplexSlot,ps.CurveNetwork]] = {}
        self.pointclouds:dict[int,tuple[SimplicialComplexSlot,ps.PointCloud]] = {}
        pass
    
    def register(self)->None:
        for geo_slot in self.scene_visitor.geometries():
            if isinstance(geo_slot, SimplicialComplexSlot):
                geo:SimplicialComplex = geo_slot.geometry()
                if geo.dim() == 3:
                    self._register_tetmesh(geo_slot)
                elif geo.dim() == 2:
                    self._register_trimesh(geo_slot)
                elif geo.dim() == 1:
                    self._register_linemesh(geo_slot)
                elif geo.dim() == 0:
                    self._register_pointcloud(geo_slot)
        pass
    
    def _register_tetmesh(self, geo_slot:SimplicialComplexSlot):
        id = geo_slot.id()
        geo = geo_slot.geometry()
        is_surf = geo.tetrahedra().find(builtin.is_surf)
        if is_surf is not None:
            geo = self.process_instance(geo)
            ps_volume_mesh = ps.register_volume_mesh(f'{id}',
                geo.positions().view().reshape(-1,3),
                geo.tetrahedra().topo().view().reshape(-1,4))
            self.tetmeshes[id] = (geo_slot, ps_volume_mesh)
    
    def _register_trimesh(self, geo_slot:SimplicialComplexSlot):
        id = geo_slot.id()
        geo = geo_slot.geometry()
        is_surf = geo.triangles().find(builtin.is_surf)
        if is_surf is not None:
            geo = self.process_instance(geo)
            ps_trimesh = ps.register_surface_mesh(f'{id}',
                geo.positions().view().reshape(-1,3),
                geo.triangles().topo().view().reshape(-1,3))
            self.trimeshes[id] =(geo_slot, ps_trimesh)
    
    def _register_linemesh(self, geo_slot:SimplicialComplexSlot):
        id = geo_slot.id()
        geo = geo_slot.geometry()
        is_surf = geo.edges().find(builtin.is_surf)
        if is_surf is not None:
            geo = self.process_instance(geo)
            ps_curve_network = ps.register_curve_network(
                f'{id}',
                geo.positions().view().reshape(-1,3),
                geo.edges().topo().view().reshape(-1,2))
            thickness = geo.vertices().find(builtin.thickness)
            if thickness is not None and geo.vertices().size() > 0:
                ps_curve_network.set_radius(thickness.view()[0], relative=False)
            
            self.linemeshes[id] = (geo_slot, ps_curve_network)
    
    def _register_pointcloud(self, geo_slot:SimplicialComplexSlot):
        id = geo_slot.id()
        geo = geo_slot.geometry()
        is_surf = geo.vertices().find(builtin.is_surf)
        if is_surf is not None:
            geo = self.process_instance(geo)
            ps_ponit_cloud = ps.register_point_cloud(
                f'{id}',
                geo.positions().view().reshape(-1,3))
            thickness = geo.vertices().find(builtin.thickness)
            if thickness is not None:
                ps_ponit_cloud.add_scalar_quantity('thickness', thickness.view())
                ps_ponit_cloud.set_point_radius_quantity('thickness', False)
            self.pointclouds[id] = (geo_slot, ps_ponit_cloud)
    
    def process_instance(self, geo:SimplicialComplex):
        if geo.instances().size() >= 1:
            split_geos:list[SimplicialComplex] = apply_transform(geo)
            return merge(split_geos)
        else:
            return geo
    
    def update(self):
        for id, (geo_slot, ps_mesh) in self.tetmeshes.items():
            geo = geo_slot.geometry()
            geo = self.process_instance(geo)
            ps_mesh.update_vertex_positions(geo.positions().view().reshape(-1,3))
        for id, (geo_slot, ps_mesh) in self.trimeshes.items():
            geo = geo_slot.geometry()
            geo = self.process_instance(geo)
            ps_mesh.update_vertex_positions(geo.positions().view().reshape(-1,3))
        for id, (geo_slot, ps_mesh) in self.linemeshes.items():
            geo = geo_slot.geometry()
            geo = self.process_instance(geo)
            ps_mesh.update_node_positions(geo.positions().view().reshape(-1,3))
        for id, (geo_slot, ps_mesh) in self.pointclouds.items():
            geo = geo_slot.geometry()
            geo = self.process_instance(geo)
            ps_mesh.update_point_positions(geo.positions().view().reshape(-1,3))
        pass
    
    def set_edge_width(self, width):
        for id, (geo_slot, ps_mesh) in self.tetmeshes.items():
            ps_mesh.set_edge_width(width)
        for id, (geo_slot, ps_mesh) in self.trimeshes.items():
            ps_mesh.set_edge_width(width)

class SceneGUI:
    def __init__(self, scene:Scene, surf_type:Literal['merge', 'split']='merge'):
        self.scene = scene
        self.surf_type = surf_type  # default type for registration, can be 'merge' or 'split'
        if surf_type == 'merge':
            self.gui = _SceneGUIMerge(scene)
        elif surf_type == 'split':
            self.gui = _SceneGUISplit(scene)
    
    def register(self, ground_name='ground')->tuple[ps.SurfaceMesh, ps.CurveNetwork, ps.PointCloud] | None:
        self._set_ground(ground_name)
        return self.gui.register()
    
    def _set_ground(self, ground_name):
        objs = self.scene.objects().find(ground_name)
        if len(objs) == 0:
            return
        ground_obj:core.Object = objs[0]
        ids = ground_obj.geometries().ids()
        if len(ids) == 0:
            return
        id = ids[0]
        geo_slot, rest_geo_slot = self.scene.geometries().find(id)
        if geo_slot is None:
            return
        P = geo_slot.geometry().instances().find('P')
        N = geo_slot.geometry().instances().find('N')
        if P is None:
            return
        if N is None:
            return
        height = P.view()[0][1]
        normal = N.view()[0]
        if (normal != np.array([0, 1, 0])).all():
            return
        ps.set_ground_plane_height(height)

    def update(self):
        self.gui.update()
    
    def set_edge_width(self, width:float):
        self.gui.set_edge_width(width)
