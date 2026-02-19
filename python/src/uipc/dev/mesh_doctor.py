import numpy as np
import uipc
import polyscope as ps
from polyscope import imgui
import os

from uipc.constitution import IConstitution, AffineBodyConstitution
from uipc.geometry import SimplicialComplex, SimplicialComplexIO, linemesh, label_open_edge

class MeshDoctor:
    def __init__(self, workspace: str, with_gui: bool = False):
        self.workspace = workspace
        self.with_gui = with_gui
        if with_gui:
            if not ps.is_initialized():
                ps.init()
    
    def check_mesh(self, constitution:IConstitution, mesh:SimplicialComplex):
        if constitution.type() == uipc.builtin.AffineBody:
            return self.check_affine_body(constitution, mesh)
        elif constitution.type() == uipc.builtin.FiniteElement:
            # TODO: Implement finite element mesh check
            return True
        else:
            raise ValueError(f'Unsupported constitution type: {constitution.type()}')
    
    def check_affine_body(self, constitution:IConstitution, mesh:SimplicialComplex):
        uipc.Logger.info(f'Checking affine body mesh: {mesh}')
        
        success = True

        if mesh.dim() == 2:
            is_open_attr = label_open_edge(mesh)
            is_open_view = is_open_attr.view()
            open_edge_count = np.sum(is_open_view)
            
            if open_edge_count > 0:
                uipc.Logger.error(f'2D Mesh for AffineBodyConstitution must be closed, '
                f'but this mesh has {open_edge_count} open edges')
                
                open_edges_mesh = self._extract_open_edges_mesh(mesh, is_open_view)
                
                io = SimplicialComplexIO()
                base_mesh_path = os.path.join(self.workspace, 'base_mesh.obj')
                open_edges_path = os.path.join(self.workspace, 'open_edges.obj')
                io.write(base_mesh_path, mesh)
                io.write(open_edges_path, open_edges_mesh)
                uipc.Logger.info(f'Written base mesh to: {base_mesh_path}')
                uipc.Logger.info(f'Written open edges to: {open_edges_path}')
                
                if self.with_gui:
                    self._visualize_open_edges(mesh, open_edges_mesh)

                success = False
        if success:
            uipc.Logger.info('Passed mesh check for AffineBodyConstitution')
        else:
            if self.with_gui:
                ps.show()
            uipc.Logger.error('Failed mesh check for AffineBodyConstitution')
        return success
    
    def _extract_open_edges_mesh(self, mesh:SimplicialComplex, is_open_view) -> SimplicialComplex:
        '''Extract open edges into a separate mesh, keeping all vertices'''
        pos_view = mesh.positions().view()
        edge_view = mesh.edges().topo().view()
        open_edge_indices = np.where(is_open_view == 1)[0]
        
        if len(open_edge_indices) == 0:
            positions = np.array(pos_view, dtype=np.float32).reshape(-1, 3)
            empty_edges = np.array([], dtype=np.int32).reshape(0, 2)
            return linemesh(positions, empty_edges)
        
        positions = np.array(pos_view, dtype=np.float32).reshape(-1, 3)
        open_edges = np.array(edge_view[open_edge_indices], dtype=np.int32).reshape(-1, 2)
        return linemesh(positions, open_edges)
    
    def _visualize_open_edges(self, mesh:SimplicialComplex, open_edges_mesh:SimplicialComplex):
        '''Visualize open edges in polyscope, extracting only vertices referenced by open edges'''
        mesh_name = 'Affine Body Mesh'
        ps_mesh = ps.register_surface_mesh(
            mesh_name,
            mesh.positions().view().reshape(-1, 3),
            mesh.triangles().topo().view().reshape(-1, 3)
        )
        ps_mesh.set_transparency(0.7)
        
        edge_view = open_edges_mesh.edges().topo().view()
        pos_view = open_edges_mesh.positions().view()
        edges = np.array(edge_view, dtype=np.int32).reshape(-1, 2)
        
        used_vertices = np.unique(edges.flatten())
        vertex_remap = {old_idx: new_idx for new_idx, old_idx in enumerate(used_vertices)}
        positions = np.array(pos_view[used_vertices], dtype=np.float32).reshape(-1, 3)
        remapped_edges = np.array([[vertex_remap[v0], vertex_remap[v1]] for v0, v1 in edges], dtype=np.int32).reshape(-1, 2)
        
        edges_name = 'Open Edges'
        ps_edges = ps.register_curve_network(
            edges_name,
            positions,
            remapped_edges,
            radius=None,
            color=[1.0, 0.0, 0.0]
        )
        ps_edges.set_enabled(True)