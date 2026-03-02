"""Download and load UIPC simulation assets from the HuggingFace dataset.

Supports both remote assets (downloaded from HuggingFace) and local asset
directories for development and testing.

Usage::

    from uipc.assets import list_assets, asset_path, load, show

    # See what's available (remote)
    print(list_assets())

    # See what's available (local)
    print(list_assets(local_dir='path/to/uipc-assets/assets'))

    # Get the local path (downloads on first call, cached afterwards)
    path = asset_path("cube_ground")

    # Or load directly into a Scene
    from uipc import Scene
    scene = Scene(Scene.default_config())
    load("cube_ground", scene)

    # Interactive viewer with asset selector
    show()                               # browse and pick any asset
    show("cube_ground")                  # preload a scene (geometry only)
    show("cube_ground", backend="cuda")  # preload and run simulation

    # Work with a local assets directory
    show(local_dir='path/to/uipc-assets/assets')
"""

import importlib.util
import pathlib

from huggingface_hub import HfApi, RepoFolder, snapshot_download
from uipc import Scene, SceneIO, Vector3, view
from uipc.geometry import SimplicialComplex, SimplicialComplexIO

REPO_ID = 'MuGdxy/uipc-assets'

__all__ = [
    'REPO_ID', 'list_assets', 'asset_path', 'read_mesh',
    'load', 'show', 'strip_constitutions',
]

_CONSTITUTION_META_ATTRS = [
    'constitution_uid',
    'extra_constitution_uids',
    'constraint_uids',
]


def strip_constitutions(scene: Scene) -> None:
    """Remove all constitution / constraint / contact metadata from scene geometries.

    After calling this, the scene contains only raw geometry (positions,
    topology, surface labels, transforms) with no physics attached.
    Useful for re-assigning constitutions or geometry-only visualization.

    Args:
        scene: A :class:`uipc.Scene` whose geometries have already been
               populated by ``build_scene`` or ``load``.
    """
    for obj_id in range(scene.objects().size()):
        obj = scene.objects().find(obj_id)
        if obj is None:
            continue
        for geo_id in obj.geometries().ids():
            geo_slot, _ = scene.geometries().find(geo_id)
            if geo_slot is None:
                continue
            geo = geo_slot.geometry()
            meta = geo.meta()
            for attr_name in _CONSTITUTION_META_ATTRS:
                if meta.find(attr_name) is not None:
                    meta.destroy(attr_name)


def list_assets(
    *,
    revision: str = 'main',
    local_dir: str | pathlib.Path | None = None,
) -> list[str]:
    """List all available asset names.

    When *local_dir* is given, scans that directory for sub-directories
    containing a ``scene.py`` file.  Otherwise queries the HuggingFace
    dataset.

    Args:
        revision: Git revision (only used for remote assets).
        local_dir: Path to a local assets directory (e.g.
            ``'uipc-assets/assets'``).  Each immediate sub-directory that
            contains a ``scene.py`` is treated as an asset.

    Returns:
        Sorted list of asset names (e.g. ``['cube_ground', 'fem_link_drop', ...]``).
    """
    if local_dir is not None:
        root = pathlib.Path(local_dir)
        if not root.is_dir():
            raise FileNotFoundError(f'Local assets directory not found: {root}')
        return sorted(
            d.name
            for d in root.iterdir()
            if d.is_dir() and (d / 'scene.py').exists()
        )

    api = HfApi()
    entries = api.list_repo_tree(
        REPO_ID, repo_type='dataset', path_in_repo='assets', revision=revision
    )
    return sorted(
        e.path.removeprefix('assets/')
        for e in entries
        if isinstance(e, RepoFolder)
    )


def read_mesh(io: SimplicialComplexIO, filepath: str | pathlib.Path) -> SimplicialComplex:
    """Read a mesh file and tag the geometry with its repo-relative path.

    Sets ``meta.file`` to ``assets/<asset_name>/<filename>`` so that
    downstream code can identify which file a geometry came from.

    Args:
        io: A :class:`~uipc.geometry.SimplicialComplexIO` instance.
        filepath: Path to the mesh file (e.g. ``ASSET_DIR / 'cube.msh'``).

    Returns:
        The loaded :class:`~uipc.geometry.SimplicialComplex`.
    """
    filepath = pathlib.Path(filepath)
    sc = io.read(str(filepath))
    rel = f'assets/{filepath.parent.name}/{filepath.name}'
    sc.meta().create('file', rel)
    return sc


def asset_path(
    name: str,
    *,
    revision: str = 'main',
    cache_dir: str | None = None,
    local_dir: str | pathlib.Path | None = None,
) -> pathlib.Path:
    """Return the local path to an asset directory.

    When *local_dir* is given, returns ``local_dir / name`` directly.
    Otherwise downloads from HuggingFace (cached by ``huggingface_hub``).

    Args:
        name: Asset name (e.g. ``'cube_ground'``).
        revision: Git revision (only used for remote assets).
        cache_dir: Where to cache downloaded files (remote only).
        local_dir: Path to a local assets directory.  When set, no
            network access is performed.

    Returns:
        :class:`~pathlib.Path` to the asset directory.
    """
    if local_dir is not None:
        result = pathlib.Path(local_dir) / name
        if not result.is_dir():
            raise FileNotFoundError(
                f'Asset \'{name}\' not found in local directory {local_dir}.'
            )
        return result

    dl = snapshot_download(
        REPO_ID,
        allow_patterns=[f'assets/{name}/**', 'assets/_*.py'],
        revision=revision,
        cache_dir=cache_dir,
        repo_type='dataset',
    )
    result = pathlib.Path(dl) / 'assets' / name
    if not result.is_dir():
        raise FileNotFoundError(
            f'Asset \'{name}\' not found in {REPO_ID}.  '
            f'Run list_assets() to see available names.'
        )
    return result


def load(
    name: str,
    scene: Scene,
    *,
    geometry_only: bool = False,
    revision: str = 'main',
    cache_dir: str | None = None,
    local_dir: str | pathlib.Path | None = None,
) -> None:
    """Download (or locate locally) an asset and apply it to a Scene.

    Imports the asset's ``scene.py`` and calls ``build_scene(scene)``.
    Each asset's ``build_scene`` may modify ``scene.config()`` to set
    parameters like ``dt`` or ``contact/d_hat``.

    Args:
        name: Asset name (e.g. ``'cube_ground'``).
        scene: A :class:`uipc.Scene` instance to populate.
        geometry_only: If ``True``, strip all constitution / constraint /
            contact metadata after building, leaving only raw geometry.
        revision: Git revision (only used for remote assets).
        cache_dir: Where to cache downloaded files (remote only).
        local_dir: Path to a local assets directory.  When set, no
            network access is performed.
    """
    path = asset_path(name, revision=revision, cache_dir=cache_dir,
                      local_dir=local_dir)
    scene_file = path / 'scene.py'
    if not scene_file.exists():
        raise FileNotFoundError(
            f'Asset \'{name}\' has no scene.py at {scene_file}'
        )

    spec = importlib.util.spec_from_file_location(f'uipc_asset_{name}', scene_file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    if not hasattr(mod, 'build_scene'):
        raise AttributeError(
            f'Asset \'{name}\' scene.py must define a build_scene(scene) function'
        )
    mod.build_scene(scene)

    if geometry_only:
        strip_constitutions(scene)


def _show_scene_config(imgui, cfg) -> None:
    """Render all scene config attributes as ImGui controls."""

    def _float(key: str, label: str, fmt: str = '%.6f'):
        attr = cfg.find(key)
        if attr is None:
            return
        val = float(view(attr)[0])
        changed, new_val = imgui.InputFloat(label, val, 0.0, 0.0, fmt)
        if changed:
            view(attr)[0] = new_val

    def _int(key: str, label: str, step: int = 1, step_fast: int = 10):
        attr = cfg.find(key)
        if attr is None:
            return
        val = int(view(attr)[0])
        changed, new_val = imgui.InputInt(label, val, step, step_fast)
        if changed:
            view(attr)[0] = new_val

    def _bool(key: str, label: str):
        attr = cfg.find(key)
        if attr is None:
            return
        val = bool(int(view(attr)[0]))
        changed, new_val = imgui.Checkbox(label, val)
        if changed:
            view(attr)[0] = int(new_val)

    def _vec3(key: str, label: str):
        attr = cfg.find(key)
        if attr is None:
            return
        g = view(attr)[0]
        gv = [float(g[0][0]), float(g[1][0]), float(g[2][0])]
        changed, new_g = imgui.DragFloat3(label, gv, 0.1, -100.0, 100.0, '%.2f')
        if changed:
            view(attr)[0] = Vector3.Values(new_g)

    def _str(key: str, label: str):
        attr = cfg.find(key)
        if attr is None:
            return
        val = str(view(attr)[0])
        imgui.InputText(label, val, imgui.ImGuiInputTextFlags_ReadOnly)

    # ── Time & Physics ────────────────────────────────────────
    if imgui.TreeNode('Time & Physics'):
        _float('dt', 'dt')
        _vec3('gravity', 'gravity')
        imgui.TreePop()

    # ── Contact ───────────────────────────────────────────────
    if imgui.TreeNode('Contact'):
        _bool('contact/enable', 'enable')
        _bool('contact/friction/enable', 'friction')
        _str('contact/constitution', 'constitution')
        _float('contact/d_hat', 'd_hat', '%.6e')
        _float('contact/eps_velocity', 'eps_velocity', '%.6e')
        _float('contact/adaptive/min_kappa', 'min_kappa', '%.6e')
        _float('contact/adaptive/init_kappa', 'init_kappa', '%.6e')
        _float('contact/adaptive/max_kappa', 'max_kappa', '%.6e')
        imgui.TreePop()

    # ── Newton Solver ─────────────────────────────────────────
    if imgui.TreeNode('Newton Solver'):
        _int('newton/max_iter', 'max_iter')
        _int('newton/min_iter', 'min_iter')
        _bool('newton/use_adaptive_tol', 'adaptive_tol')
        _float('newton/velocity_tol', 'velocity_tol', '%.6e')
        _float('newton/ccd_tol', 'ccd_tol', '%.4f')
        _float('newton/transrate_tol', 'transrate_tol', '%.6e')
        _bool('newton/semi_implicit/enable', 'semi_implicit')
        _float('newton/semi_implicit/beta_tol', 'beta_tol', '%.6e')
        imgui.TreePop()

    # ── Linear System ─────────────────────────────────────────
    if imgui.TreeNode('Linear System'):
        _float('linear_system/tol_rate', 'tol_rate', '%.6e')
        _str('linear_system/solver', 'solver')
        _bool('linear_system/precond/mas/contact_aware', 'contact_aware')
        imgui.TreePop()

    # ── Line Search ───────────────────────────────────────────
    if imgui.TreeNode('Line Search'):
        _int('line_search/max_iter', 'max_iter')
        _bool('line_search/report_energy', 'report_energy')
        imgui.TreePop()

    # ── Misc ──────────────────────────────────────────────────
    if imgui.TreeNode('Misc'):
        _str('integrator/type', 'integrator')
        _bool('cfl/enable', 'CFL')
        _str('collision_detection/method', 'collision method')
        _bool('sanity_check/enable', 'sanity_check')
        _str('sanity_check/mode', 'sanity_mode')
        _bool('diff_sim/enable', 'diff_sim')
        _bool('extras/strict_mode/enable', 'strict_mode')
        _bool('extras/debug/dump_surface', 'dump_surface')
        _bool('extras/debug/dump_linear_system', 'dump_linear_sys')
        _bool('extras/debug/dump_linear_pcg', 'dump_linear_pcg')
        imgui.TreePop()


def _show_contact_tabular(imgui, ct) -> None:
    """Render editable contact tabular as ImGui controls."""
    n = ct.element_count()

    # Build element name lookup: id → name.
    de = ct.default_element()
    elem_names = {de.id(): de.name() or 'default'}

    imgui.TextUnformatted(f'Elements: {n}')

    # Show element list with names.
    if imgui.TreeNode('Elements'):
        for i in range(n):
            name = elem_names.get(i, f'element_{i}')
            imgui.TextUnformatted(f'  [{i}] {name}')
        imgui.TreePop()

    # ── Default model (editable) ──────────────────────────────
    def_name = elem_names.get(de.id(), 'default')
    if imgui.TreeNode(f'Default model ({def_name})'):
        dm = ct.default_model()
        fr = dm.friction_rate()
        res = dm.resistance()
        ena = dm.is_enabled()

        ch_f, new_fr = imgui.InputFloat('friction##def', fr, 0.0, 0.0, '%.4f')
        ch_r, new_res = imgui.InputFloat('resistance##def', res, 0.0, 0.0, '%.2e')
        ch_e, new_ena = imgui.Checkbox('enabled##def', ena)

        if ch_f or ch_r or ch_e:
            ct.default_model(
                new_fr if ch_f else fr,
                new_res if ch_r else res,
                new_ena if ch_e else ena,
            )
        imgui.TreePop()

    # ── Pairwise models (editable) ────────────────────────────
    if n > 1 and imgui.TreeNode('Pairwise models'):
        for i in range(n):
            for j in range(i, n):
                m = ct.at(i, j)
                ni = elem_names.get(i, f'element_{i}')
                nj = elem_names.get(j, f'element_{j}')
                tag = f'##{i}_{j}'
                header = f'{ni} <-> {nj}{tag}'
                if imgui.TreeNode(header):
                    fr = m.friction_rate()
                    res = m.resistance()
                    ena = m.is_enabled()

                    ch_f, new_fr = imgui.InputFloat(
                        f'friction{tag}', fr, 0.0, 0.0, '%.4f',
                    )
                    ch_r, new_res = imgui.InputFloat(
                        f'resistance{tag}', res, 0.0, 0.0, '%.2e',
                    )
                    ch_e, new_ena = imgui.Checkbox(f'enabled{tag}', ena)

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
                    imgui.TreePop()
        imgui.TreePop()


def show(
    name: str | None = None,
    backend: str = 'none',
    *,
    gui: bool = True,
    workspace: str | None = None,
    revision: str = 'main',
    cache_dir: str | None = None,
    local_dir: str | pathlib.Path | None = None,
    distance_factor: float = 2.0,
) -> None:
    """Display and optionally simulate a UIPC asset in a Polyscope window.

    Opens an interactive viewer with an **Asset Selector** panel that lists
    every available asset.  You can pick any scene and hot-load it without
    restarting.  When *backend* is a real engine (e.g. ``'cuda'``), a
    **Run / Pause** control is shown as well.

    Supports both remote assets (from HuggingFace) and local asset
    directories for development::

        from uipc.assets import show

        show()                               # browse remote assets
        show('cube_ground')                  # preload one scene
        show('cube_ground', backend='cuda')  # preload + live simulation
        show(backend='cuda')                 # browse, simulate what you pick
        show(local_dir='uipc-assets/assets') # browse local assets

    Args:
        name: Optional asset name to preload (e.g. ``'cube_ground'``).
            If ``None``, the viewer opens empty and you pick from the list.
        backend: Engine backend (default ``'none'`` for geometry-only preview,
            ``'cuda'`` for live simulation).
        gui: If ``True`` (default), open a Polyscope window.
            If ``False`` with a real backend, run headlessly.
        workspace: Directory for simulation output.  ``None`` uses a
            temporary directory.
        revision: Git revision (only used for remote assets).
        cache_dir: Where to cache downloaded files (remote only).
        local_dir: Path to a local assets directory.  When set, assets
            are loaded from disk without any network access.
        distance_factor: How far the camera sits relative to the bounding-box
            diagonal (default ``2.0``).
    """
    import tempfile
    import threading

    from uipc import Engine, World

    is_simulation = (backend != 'none')

    # ── headless mode (gui=False with a real backend) ──────────────
    if not gui:
        if name is None:
            raise ValueError(
                'Headless mode (gui=False) requires a scene name.'
            )
        scene = Scene(Scene.default_config())
        load(name, scene, revision=revision, cache_dir=cache_dir,
             local_dir=local_dir)
        ws = workspace or tempfile.mkdtemp(prefix=f'uipc_{name}_')
        engine = Engine(backend, ws)
        world = World(engine)
        world.init(scene)
        if not world.is_valid():
            raise RuntimeError(
                f"Scene '{name}' failed sanity check, world is not valid."
            )
        while world.is_valid():
            world.advance()
            world.sync()
            world.retrieve()
        return

    # ── GUI mode ───────────────────────────────────────────────────
    import polyscope as ps
    import polyscope.imgui as imgui
    from uipc.gui import SceneGUI

    # Mutable state shared with the ImGui callback.
    state = {
        'scene': None,
        'engine': None,
        'world': None,
        'scene_gui': None,
        'current_name': None,
        'running': False,
        'frame': 0,
        'selected_idx': 0,
        'asset_names': None,   # None = not yet fetched
        'fetching': False,
        'status': '',
        'error': '',
    }

    # ── helper: (re-)load a scene into Polyscope ──────────────────
    def _load_scene(asset_name: str) -> None:
        try:
            ps.remove_all_structures()

            scene = Scene(Scene.default_config())
            load(asset_name, scene, revision=revision, cache_dir=cache_dir,
                 local_dir=local_dir)

            if is_simulation:
                ws = workspace or tempfile.mkdtemp(
                    prefix=f'uipc_{asset_name}_'
                )
                engine = Engine(backend, ws)
            else:
                engine = Engine('none')
            world = World(engine)
            world.init(scene)

            scene_gui = SceneGUI(scene, surf_type='split')
            scene_gui.register()
            scene_gui.set_edge_width(1.0)
            _auto_camera(scene, distance_factor)

            state['scene'] = scene
            state['engine'] = engine
            state['world'] = world
            state['scene_gui'] = scene_gui
            state['current_name'] = asset_name
            state['running'] = False
            state['frame'] = 0
            state['status'] = f'Loaded: {asset_name}'
            state['error'] = ''
        except Exception as exc:
            state['error'] = str(exc)
            state['status'] = f'Error loading {asset_name}'

    # ── helper: fetch asset list in a background thread ───────────
    def _fetch_assets() -> None:
        try:
            names = list_assets(revision=revision, local_dir=local_dir)
            state['asset_names'] = names
            # Pre-select the currently loaded scene, if any.
            if state['current_name'] and state['current_name'] in names:
                state['selected_idx'] = names.index(state['current_name'])
        except Exception as exc:
            state['asset_names'] = []
            state['error'] = f'Failed to fetch asset list: {exc}'
        finally:
            state['fetching'] = False

    # ── ImGui callback ────────────────────────────────────────────
    def _callback() -> None:
        # ---- Asset Selector section ----
        imgui.TextUnformatted('Asset Selector')
        imgui.Separator()

        names = state['asset_names']

        # Kick off the background fetch once.
        if names is None and not state['fetching']:
            state['fetching'] = True
            threading.Thread(target=_fetch_assets, daemon=True).start()

        if state['fetching']:
            imgui.TextUnformatted('Fetching asset list...')
        elif names is not None and len(names) > 0:
            # Scrollable list of assets.
            line_h = imgui.GetTextLineHeightWithSpacing()
            list_h = min(len(names), 15) * line_h
            if imgui.BeginListBox('##assets', (0.0, list_h)):
                for i, n in enumerate(names):
                    is_cur = (i == state['selected_idx'])
                    if imgui.Selectable(n, selected=is_cur):
                        state['selected_idx'] = i
                imgui.EndListBox()

            # Read-only text box showing the selected name (user can copy).
            sel = names[state['selected_idx']]
            imgui.InputText('##selected', sel,
                            imgui.ImGuiInputTextFlags_ReadOnly)

            if imgui.Button('Load'):
                state['status'] = f'Loading {sel}...'
                _load_scene(sel)
        elif names is not None:
            imgui.TextUnformatted('No assets found.')

        # Status / error feedback.
        if state['status']:
            imgui.TextUnformatted(state['status'])
        if state['error']:
            imgui.TextColored((1.0, 0.4, 0.4, 1.0), state['error'])

        # ---- Scene Parameters section ----
        if state['scene'] is not None:
            imgui.Separator()
            if imgui.CollapsingHeader('Scene Config'):
                scene = state['scene']
                cfg = scene.config()
                _show_scene_config(imgui, cfg)

            if imgui.CollapsingHeader('Contact Tabular'):
                scene = state['scene']
                ct = scene.contact_tabular()
                _show_contact_tabular(imgui, ct)

            # Re-init world with updated parameters
            if is_simulation and state['world'] is not None:
                if imgui.Button('Re-init'):
                    try:
                        scene = state['scene']
                        ws = workspace or tempfile.mkdtemp(
                            prefix=f'uipc_{state["current_name"]}_'
                        )
                        engine = Engine(backend, ws)
                        world = World(engine)
                        world.init(scene)
                        ps.remove_all_structures()
                        scene_gui = SceneGUI(scene, surf_type='split')
                        scene_gui.register()
                        scene_gui.set_edge_width(1.0)
                        state['engine'] = engine
                        state['world'] = world
                        state['scene_gui'] = scene_gui
                        state['running'] = False
                        state['frame'] = 0
                        state['status'] = f'Re-init: {state["current_name"]}'
                        state['error'] = ''
                    except Exception as exc:
                        state['error'] = str(exc)

        # ---- Simulation controls (only for real backends) ----
        if is_simulation and state['world'] is not None:
            imgui.Separator()
            _, state['running'] = imgui.Checkbox(
                'Run', state['running']
            )
            imgui.SameLine()
            imgui.TextUnformatted(f"frame: {state['frame']}")

            if state['running'] and state['world'].is_valid():
                state['world'].advance()
                state['world'].sync()
                state['world'].retrieve()
                state['scene_gui'].update()
                state['frame'] += 1

    # ── Initialise Polyscope and (optionally) preload a scene ─────
    ps.init()

    if name is not None:
        _load_scene(name)

    ps.set_user_callback(_callback)
    ps.show()


def _auto_camera(scene: Scene, distance_factor: float) -> None:
    """Position the camera looking down at 45 deg based on the scene bbox."""
    import numpy as np
    import polyscope as ps

    sio = SceneIO(scene)
    all_pts: list[np.ndarray] = []
    for dim in (0, 1, 2):
        surf = sio.simplicial_surface(dim)
        if surf.vertices().size() > 0:
            all_pts.append(surf.positions().view().reshape(-1, 3))

    if not all_pts:
        return

    pts = np.concatenate(all_pts, axis=0)
    center = (pts.min(axis=0) + pts.max(axis=0)) * 0.5
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))

    if diag < 1e-12:
        return

    dist = diag * distance_factor
    # 45-deg elevation, looking from the (+X, +Y, +Z) octant toward center
    elevation = dist * np.sin(np.pi / 4)
    ground = dist * np.cos(np.pi / 4) / np.sqrt(2)
    camera_pos = center + np.array([ground, elevation, ground])
    ps.look_at(camera_pos.tolist(), center.tolist())
