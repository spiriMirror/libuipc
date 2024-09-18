import pyuipc_loader
from pyuipc_loader import pyuipc
import polyscope as ps
from pyuipc import Scene, SceneIO

class SceneGUI:
    def __init__(self, scene:Scene):
        self.scene_io = SceneIO(scene)
        self.ps_trimesh:ps.SurfaceMesh = None
        self.ps_linemesh:ps.CurveNetwork = None
        self.ps_poincloud:ps.PointCloud = None
    
    def register(self)->tuple[ps.SurfaceMesh, ps.CurveNetwork, ps.PointCloud]:
        trimesh = self.scene_io.simplicial_surface(2)
        linemesh = self.scene_io.simplicial_surface(1)
        pointcloud = self.scene_io.simplicial_surface(0)
        if(trimesh.vertices().size() != 0):
            self.ps_trimesh = ps.register_surface_mesh('trimesh', trimesh.positions().view().reshape(-1,3), trimesh.triangles().topo().view().reshape(-1,3))
        if(linemesh.vertices().size() != 0):
            self.ps_linemesh = ps.register_curve_network('linemesh', linemesh.positions().view().reshape(-1,3), linemesh.edges().topo().view().reshape(-1,2))
        if(pointcloud.vertices().size() != 0):
            self.ps_poincloud = ps.register_point_cloud('pointcloud', pointcloud.positions().view().reshape(-1,3))
            thickness = pointcloud.vertices().find(pyuipc.builtin.thickness).view()
            self.ps_poincloud.add_scalar_quantity('thickness', thickness)
            self.ps_poincloud.set_point_radius_quantity('thickness', False)
        return self.ps_trimesh, self.ps_linemesh, self.ps_poincloud
    
    def update(self):
        if self.ps_trimesh is not None:
            trimesh = self.scene_io.simplicial_surface(2)
            self.ps_trimesh.update_vertex_positions(trimesh.positions().view().reshape(-1,3))
        if self.ps_linemesh is not None:
            linemesh = self.scene_io.simplicial_surface(1)
            self.ps_linemesh.update_node_positions(linemesh.positions().view().reshape(-1,3))
        if self.ps_poincloud is not None:
            pointcloud = self.scene_io.simplicial_surface(0)
            self.ps_poincloud.update_point_positions(pointcloud.positions().view().reshape(-1,3))
