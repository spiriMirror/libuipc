"""Download and load UIPC simulation assets from the HuggingFace dataset.

Supports both remote assets (downloaded from HuggingFace) and local asset
directories for development and testing.

Usage::

    from uipc.assets import list_assets, asset_path, load, show

    # See what's available (remote)
    print(list_assets())

    # See what's available (local)
    print(list_assets(local_repo='path/to/uipc-assets'))

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
    show(local_repo='path/to/uipc-assets')
"""

import importlib.util
import pathlib

from huggingface_hub import HfApi, RepoFolder, snapshot_download
from uipc import Scene, SceneIO
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
    local_repo: str | pathlib.Path | None = None,
) -> list[str]:
    """List all available asset names.

    When *local_repo* is given, scans that directory's ``assets``
    sub-directory for asset folders
    containing a ``scene.py`` file.  Otherwise queries the HuggingFace
    dataset.

    Args:
        revision: Git revision (only used for remote assets).
        local_repo: Path to local asset repo root (e.g. ``'uipc-assets'``).
            Assets are discovered under ``local_repo / 'assets'``.

    Returns:
        Sorted list of asset names (e.g. ``['cube_ground', 'fem_link_drop', ...]``).
    """
    root = _resolve_local_assets_dir(local_repo=local_repo)
    if root is not None:
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
    local_repo: str | pathlib.Path | None = None,
) -> pathlib.Path:
    """Return the local path to an asset directory.

    When *local_repo* is given, returns ``local_repo / 'assets' / name`` directly.
    Otherwise downloads from HuggingFace (cached by ``huggingface_hub``).

    Args:
        name: Asset name (e.g. ``'cube_ground'``).
        revision: Git revision (only used for remote assets).
        cache_dir: Where to cache downloaded files (remote only).
        local_repo: Path to local asset repo root. When set, assets are
            loaded from ``local_repo / 'assets'`` and no network access is
            performed.

    Returns:
        :class:`~pathlib.Path` to the asset directory.
    """
    local_assets_dir = _resolve_local_assets_dir(local_repo=local_repo)
    if local_assets_dir is not None:
        result = local_assets_dir / name
        if not result.is_dir():
            raise FileNotFoundError(
                f"Asset '{name}' not found in local path {local_assets_dir}."
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
    local_repo: str | pathlib.Path | None = None,
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
        local_repo: Path to local asset repo root. When set, assets are
            loaded from ``local_repo / 'assets'`` and no network access is
            performed.
    """
    path = asset_path(
        name,
        revision=revision,
        cache_dir=cache_dir,
        local_repo=local_repo,
    )
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


def show(
    name: str | None = None,
    backend: str = 'none',
    *,
    gui: bool = True,
    workspace: str | None = None,
    revision: str = 'main',
    cache_dir: str | None = None,
    local_repo: str | pathlib.Path | None = None,
    distance_factor: float = 2.0,
) -> None:
    """Display and optionally simulate a UIPC asset in a Polyscope window.

    Opens an interactive viewer with a collapsable **Asset Selector** panel
    that lists every available asset. You can pick any scene and hot-load it
    without restarting. Once loaded, the GUI shows the source directory and
    ``scene.py`` path for the current scene, and supports exporting the scene
    snapshot from a native save dialog. When *backend* is a real engine
    (e.g. ``'cuda'``), a **Run / Pause** control is shown as well.

    Supports both remote assets (from HuggingFace) and local asset
    directories for development::

        from uipc.assets import show

        show()                               # browse remote assets
        show('cube_ground')                  # preload one scene
        show('cube_ground', backend='cuda')  # preload + live simulation
        show(backend='cuda')                 # browse, simulate what you pick
        show(local_repo='uipc-assets')       # browse local assets

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
        local_repo: Path to local asset repo root. Assets are loaded from
            ``local_repo / 'assets'`` without network access.
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
        load(
            name,
            scene,
            revision=revision,
            cache_dir=cache_dir,
            local_repo=local_repo,
        )
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
        'source_dir': '',
        'scene_file': '',
        'status': '',
        'error': '',
    }

    # ── helper: (re-)load a scene into Polyscope ──────────────────
    def _load_scene(asset_name: str) -> None:
        try:
            ps.remove_all_structures()

            scene = Scene(Scene.default_config())
            source_dir = asset_path(
                asset_name,
                revision=revision,
                cache_dir=cache_dir,
                local_repo=local_repo,
            )
            scene_file = source_dir / 'scene.py'
            load(
                asset_name,
                scene,
                revision=revision,
                cache_dir=cache_dir,
                local_repo=local_repo,
            )

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
            state['source_dir'] = str(source_dir.resolve())
            state['scene_file'] = str(scene_file.resolve())
            state['status'] = f'Loaded: {asset_name}'
            state['error'] = ''
        except Exception as exc:
            state['error'] = str(exc)
            state['status'] = f'Error loading {asset_name}'

    # ── helper: fetch asset list in a background thread ───────────
    def _fetch_assets() -> None:
        try:
            names = list_assets(
                revision=revision,
                local_repo=local_repo,
            )
            state['asset_names'] = names
            # Pre-select the currently loaded scene, if any.
            if state['current_name'] and state['current_name'] in names:
                state['selected_idx'] = names.index(state['current_name'])
        except Exception as exc:
            state['asset_names'] = []
            state['error'] = f'Failed to fetch asset list: {exc}'
        finally:
            state['fetching'] = False

    def _pick_save_path(asset_name: str, source_dir: str) -> pathlib.Path | None:
        """Open a native file-save dialog and return selected path."""
        try:
            import tkinter as tk
            from tkinter import filedialog
        except Exception as exc:
            raise RuntimeError(
                f'Native file dialog is unavailable: {exc}'
            ) from exc

        initial_dir = source_dir or str(pathlib.Path.cwd())
        initial_file = f'{asset_name}.json'

        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        try:
            chosen = filedialog.asksaveasfilename(
                title='Save Scene',
                defaultextension='.json',
                filetypes=[
                    ('JSON file', '*.json'),
                    ('BSON file', '*.bson'),
                    ('All files', '*.*'),
                ],
                initialdir=initial_dir,
                initialfile=initial_file,
            )
        finally:
            root.destroy()

        if not chosen:
            return None
        return pathlib.Path(chosen)

    # ── ImGui callback ────────────────────────────────────────────
    def _callback() -> None:
        names = state['asset_names']

        # Kick off the background fetch once.
        if names is None and not state['fetching']:
            state['fetching'] = True
            threading.Thread(target=_fetch_assets, daemon=True).start()

        # ---- Asset Selector section ----
        if imgui.CollapsingHeader('Asset Selector'):
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
                imgui.InputText(
                    '##selected',
                    sel,
                    imgui.ImGuiInputTextFlags_ReadOnly,
                )

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
            def _on_save_scene() -> None:
                try:
                    save_path = _pick_save_path(
                        state['current_name'],
                        state['source_dir'],
                    )
                    if save_path is not None:
                        sio = SceneIO(state['scene'])
                        sio.save(str(save_path))
                        state['status'] = f'Saved scene: {save_path}'
                        state['error'] = ''
                    else:
                        state['status'] = 'Save canceled.'
                except Exception as exc:
                    state['error'] = f'Failed to save scene: {exc}'

            imgui.Separator()
            state['scene_gui'].show(
                imgui,
                current_name=state['current_name'],
                source_dir=state['source_dir'],
                scene_file=state['scene_file'],
                on_save_scene=_on_save_scene,
            )

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


def _resolve_local_assets_dir(
    *,
    local_repo: str | pathlib.Path | None,
) -> pathlib.Path | None:
    """Resolve local assets root directory from local_repo input."""
    if local_repo is not None:
        return pathlib.Path(local_repo) / 'assets'
    return None
