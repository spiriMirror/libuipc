import polyscope as ps
from uipc import Scene, SceneIO, Vector3, view
from uipc import builtin
from uipc import core
from uipc.backend import SceneVisitor
from uipc.geometry import SimplicialComplexSlot, SimplicialComplex, extract_surface, apply_transform, merge
from typing import Callable, Literal
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
            ps.set_ground_plane_mode("none")
            return

        ground_obj:core.Object = objs[0]
        ids = ground_obj.geometries().ids()
        if len(ids) == 0:
            ps.set_ground_plane_mode("none")
            return

        id = ids[0]
        geo_slot, rest_geo_slot = self.scene.geometries().find(id)
        if geo_slot is None:
            ps.set_ground_plane_mode("none")
            return

        P = geo_slot.geometry().instances().find('P')
        N = geo_slot.geometry().instances().find('N')
        if P is None or N is None:
            ps.set_ground_plane_mode("none")
            return

        normal = N.view()[0]
        normal = normal.flatten()
        if np.allclose(normal, [0, 1, 0]):
            ps.set_up_dir("y_up")
        elif np.allclose(normal, [1, 0, 0]):
            ps.set_up_dir("x_up")
        elif np.allclose(normal, [0, 0, 1]):
            ps.set_up_dir("z_up")
        else:
            ps.set_ground_plane_mode("none")
            return

        height = float(P.view()[0][1][0])
        ps.set_ground_plane_mode("tile_reflection")
        ps.set_ground_plane_height(height)

    def update(self):
        self.gui.update()
    
    def set_edge_width(self, width:float):
        self.gui.set_edge_width(width)
    
    def show(
        self,
        imgui,
        *,
        current_name: str | None = None,
        source_dir: str = '',
        scene_file: str = '',
        on_save_scene: Callable[[], None] | None = None,
    ) -> None:
        """Render all scene-related information panels."""
        imgui.TextUnformatted(f'Current Asset: {current_name or ""}')
        imgui.InputText(
            'Source Dir',
            source_dir,
            imgui.ImGuiInputTextFlags_ReadOnly,
        )
        imgui.InputText(
            'Scene File',
            scene_file,
            imgui.ImGuiInputTextFlags_ReadOnly,
        )

        if on_save_scene is not None and imgui.Button('Save Scene'):
            on_save_scene()

        imgui.Separator()
        
        if imgui.CollapsingHeader('Scene Config'):
            self._show_scene_config(imgui, self.scene.config())

        if imgui.CollapsingHeader('Contact Tabular'):
            self._show_contact_tabular(imgui, self.scene.contact_tabular())

        if imgui.CollapsingHeader('Animator'):
            self._show_animator_info(imgui)


    def _show_scene_config(self, imgui, cfg) -> None:
        """Render scene config by traversing config keys dynamically."""
        def _try_vec3(raw) -> list[float] | None:
            try:
                return [float(raw[0][0]), float(raw[1][0]), float(raw[2][0])]
            except Exception:
                return None

        def _show_scalar(path: str, label: str) -> None:
            attr = cfg.find(path)
            if attr is None:
                imgui.TextUnformatted(f'{label}: <missing>')
                return

            widget_label = f'{label}##{path}'
            slot = view(attr)
            raw = slot[0]
            try:
                type_name = str(attr.type_name())
            except Exception:
                type_name = ''
            t_upper = type_name.upper()

            if 'STRING' in t_upper or isinstance(raw, str):
                imgui.InputText(widget_label, raw, imgui.ImGuiInputTextFlags_ReadOnly)
                return

            if isinstance(raw, (bool, np.bool_)):
                cur = bool(raw)
                changed, new_val = imgui.Checkbox(widget_label, cur)
                if changed:
                    slot[0] = int(new_val)
                return

            if path.endswith('/enable') or path == 'enable':
                try:
                    cur = bool(int(raw))
                    changed, new_val = imgui.Checkbox(widget_label, cur)
                    if changed:
                        slot[0] = int(new_val)
                    return
                except Exception:
                    pass

            if 'VECTOR3' in t_upper and 'VECTOR3I' not in t_upper:
                vec3 = _try_vec3(raw)
                if vec3 is None:
                    imgui.InputText(
                        widget_label,
                        str(raw),
                        imgui.ImGuiInputTextFlags_ReadOnly,
                    )
                    return
                changed, new_g = imgui.DragFloat3(
                    widget_label, vec3, 0.1, -100.0, 100.0, '%.2f'
                )
                if changed:
                    slot[0] = Vector3.Values(new_g)
                return

            if (
                'FLOAT' in t_upper
                or ('VECTOR' in t_upper and 'VECTOR3I' not in t_upper and 'VECTOR2I' not in t_upper and 'VECTOR4I' not in t_upper)
                or 'MATRIX' in t_upper
                or isinstance(raw, (float, np.floating))
            ):
                if not np.isscalar(raw):
                    imgui.InputText(
                        widget_label,
                        str(raw),
                        imgui.ImGuiInputTextFlags_ReadOnly,
                    )
                    return
                cur = float(raw)
                changed, new_val = imgui.InputFloat(
                    widget_label, cur, 0.0, 0.0, '%.6g'
                )
                if changed:
                    slot[0] = new_val
                return

            if (
                any(tok in t_upper for tok in ('I32', 'I64', 'U32', 'U64', 'VECTOR2I', 'VECTOR3I', 'VECTOR4I'))
                or isinstance(raw, (int, np.integer))
            ):
                if not np.isscalar(raw):
                    imgui.InputText(
                        widget_label,
                        str(raw),
                        imgui.ImGuiInputTextFlags_ReadOnly,
                    )
                    return
                cur = int(raw)
                changed, new_val = imgui.InputInt(widget_label, cur, 1, 10)
                if changed:
                    slot[0] = new_val
                return

            val = str(raw)
            imgui.InputText(widget_label, val, imgui.ImGuiInputTextFlags_ReadOnly)

        def _build_tree(paths: list[str]) -> dict:
            root: dict = {}
            for path in paths:
                node = root
                parts = [p for p in path.split('/') if p]
                for p in parts:
                    node = node.setdefault(p, {})
                node['__path__'] = path
            return root

        preferred_order = [
            'dt',
            'gravity',
            'cfl/enable',
            'integrator/type',
            'newton/max_iter',
            'newton/min_iter',
            'newton/use_adaptive_tol',
            'newton/velocity_tol',
            'newton/ccd_tol',
            'newton/transrate_tol',
            'newton/semi_implicit/enable',
            'newton/semi_implicit/beta_tol',
            'linear_system/tol_rate',
            'linear_system/solver',
            'linear_system/precond/mas/contact_aware',
            'line_search/max_iter',
            'line_search/report_energy',
            'contact/enable',
            'contact/friction/enable',
            'contact/constitution',
            'contact/d_hat',
            'contact/adaptive/min_kappa',
            'contact/adaptive/init_kappa',
            'contact/adaptive/max_kappa',
            'contact/eps_velocity',
            'collision_detection/method',
            'sanity_check/enable',
            'sanity_check/mode',
            'diff_sim/enable',
            'extras/debug/dump_surface',
            'extras/debug/dump_linear_system',
            'extras/debug/dump_linear_pcg',
            'extras/strict_mode/enable',
        ]
        order_map = {k: i for i, k in enumerate(preferred_order)}
        unknown_order = len(preferred_order) + 1

        def _node_min_order(node: dict) -> int:
            min_order = unknown_order
            path = node.get('__path__')
            if isinstance(path, str):
                min_order = min(min_order, order_map.get(path, unknown_order))
            for key, child in node.items():
                if key == '__path__':
                    continue
                min_order = min(min_order, _node_min_order(child))
            return min_order

        def _render_tree(node: dict) -> None:
            keys = [k for k in node.keys() if k != '__path__']
            keys.sort(key=lambda k: (_node_min_order(node[k]), k))
            for key in keys:
                child = node[key]
                has_children = any(k != '__path__' for k in child.keys())
                if has_children:
                    if imgui.TreeNode(key):
                        if '__path__' in child:
                            _show_scalar(child['__path__'], key)
                        _render_tree(child)
                        imgui.TreePop()
                else:
                    _show_scalar(child.get('__path__', key), key)

        cfg_json = cfg.to_json()
        if not isinstance(cfg_json, dict):
            imgui.TextUnformatted('Config is unavailable.')
            return

        paths = [k for k in cfg_json.keys() if isinstance(k, str)]
        paths.sort(key=lambda p: (order_map.get(p, unknown_order), p))
        _render_tree(_build_tree(paths))

    def _show_contact_tabular(self, imgui, ct) -> None:
        """Render editable contact tabular as ImGui controls."""
        n = ct.element_count()
        de = ct.default_element()
        elem_names = {de.id(): de.name() or 'default'}
        flags = imgui.ImGuiTableFlags_Borders | imgui.ImGuiTableFlags_RowBg
        imgui.TextUnformatted(f'Elements: {n}')

        if imgui.BeginTable('contact_elements_table', 2, flags):
            imgui.TableSetupColumn('ID')
            imgui.TableSetupColumn('Name')
            imgui.TableHeadersRow()
            for i in range(n):
                imgui.TableNextRow()
                imgui.TableNextColumn()
                imgui.TextUnformatted(str(i))
                imgui.TableNextColumn()
                imgui.TextUnformatted(elem_names.get(i, f'element_{i}'))
            imgui.EndTable()

        dm = ct.default_model()
        fr = dm.friction_rate()
        res = dm.resistance()
        ena = dm.is_enabled()
        if imgui.BeginTable('contact_default_table', 4, flags):
            imgui.TableSetupColumn('Default')
            imgui.TableSetupColumn('Friction')
            imgui.TableSetupColumn('Resistance')
            imgui.TableSetupColumn('Enabled')
            imgui.TableHeadersRow()
            imgui.TableNextRow()
            imgui.TableNextColumn()
            imgui.TextUnformatted(elem_names.get(de.id(), 'default'))
            imgui.TableNextColumn()
            ch_f, new_fr = imgui.InputFloat('##def_friction', fr, 0.0, 0.0, '%.4f')
            imgui.TableNextColumn()
            ch_r, new_res = imgui.InputFloat('##def_resistance', res, 0.0, 0.0, '%.2e')
            imgui.TableNextColumn()
            ch_e, new_ena = imgui.Checkbox('##def_enabled', ena)
            if ch_f or ch_r or ch_e:
                ct.default_model(
                    new_fr if ch_f else fr,
                    new_res if ch_r else res,
                    new_ena if ch_e else ena,
                )
            imgui.EndTable()

        if n > 1 and imgui.BeginTable('contact_pairwise_table', 5, flags):
            imgui.TableSetupColumn('Left')
            imgui.TableSetupColumn('Right')
            imgui.TableSetupColumn('Friction')
            imgui.TableSetupColumn('Resistance')
            imgui.TableSetupColumn('Enabled')
            imgui.TableHeadersRow()
            for i in range(n):
                for j in range(i, n):
                    m = ct.at(i, j)
                    ni = elem_names.get(i, f'element_{i}')
                    nj = elem_names.get(j, f'element_{j}')
                    tag = f'##{i}_{j}'
                    fr = m.friction_rate()
                    res = m.resistance()
                    ena = m.is_enabled()

                    imgui.TableNextRow()
                    imgui.TableNextColumn()
                    imgui.TextUnformatted(ni)
                    imgui.TableNextColumn()
                    imgui.TextUnformatted(nj)
                    imgui.TableNextColumn()
                    ch_f, new_fr = imgui.InputFloat(
                        f'##friction{tag}', fr, 0.0, 0.0, '%.4f',
                    )
                    imgui.TableNextColumn()
                    ch_r, new_res = imgui.InputFloat(
                        f'##resistance{tag}', res, 0.0, 0.0, '%.2e',
                    )
                    imgui.TableNextColumn()
                    ch_e, new_ena = imgui.Checkbox(f'##enabled{tag}', ena)

                    if ch_f or ch_r or ch_e:
                        from uipc.core import ContactElement
                        L = ContactElement(i, '')
                        R = ContactElement(j, '')
                        ct.insert(
                            L, R,
                            new_fr if ch_f else fr,
                            new_res if ch_r else res,
                            new_ena if ch_e else ena,
                        )
            imgui.EndTable()

    def _show_animator_info(self, imgui) -> None:
        """Render read-only animator information in an ImGui panel."""

        def _show_animator_objects_table(items: list[tuple[int, str]]) -> None:
            imgui.TextUnformatted('animated_objects:')
            if len(items) == 0:
                imgui.TextUnformatted('  (none)')
                return
            flags = imgui.ImGuiTableFlags_Borders | imgui.ImGuiTableFlags_RowBg
            if imgui.BeginTable('animator_objects_table', 2, flags):
                imgui.TableSetupColumn('Object ID')
                imgui.TableSetupColumn('Object Name')
                imgui.TableHeadersRow()
                for oid, name in items:
                    imgui.TableNextRow()
                    imgui.TableNextColumn()
                    imgui.TextUnformatted(str(oid))
                    imgui.TableNextColumn()
                    imgui.TextUnformatted(name)
                imgui.EndTable()

        try:
            animator = self.scene.animator()
            imgui.TextUnformatted(f'substep: {int(animator.substep())}')
            if hasattr(animator, 'animation_count'):
                count = int(animator.animation_count())
                imgui.TextUnformatted(f'animation_count: {count}')
            if hasattr(animator, 'animation_ids'):
                ids = [int(i) for i in animator.animation_ids()]
                table_data: list[tuple[int, str]] = []
                for oid in ids:
                    obj = self.scene.objects().find(oid)
                    if obj is None:
                        table_data.append((oid, '<missing>'))
                    else:
                        table_data.append((oid, obj.name()))
                _show_animator_objects_table(table_data)
        except Exception as exc:
            imgui.TextUnformatted(f'Animator unavailable: {exc}')
            return

        cfg = self.scene.config()
        dt_attr = cfg.find('dt')
        if dt_attr is not None:
            dt = float(dt_attr.view()[0])
            imgui.TextUnformatted(f'dt: {dt:.6g}')
