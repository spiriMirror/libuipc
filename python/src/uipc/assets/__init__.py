"""Download and load UIPC simulation assets from the HuggingFace dataset.

Usage::

    from uipc.assets import list_assets, asset_path, load

    # See what's available
    print(list_assets())

    # Get the local path (downloads on first call, cached afterwards)
    path = asset_path("cube_ground")

    # Or load directly into a Scene
    from uipc import Scene
    scene = Scene(Scene.default_config())
    load("cube_ground", scene)
"""

import importlib.util
import pathlib

from huggingface_hub import HfApi, RepoFolder, snapshot_download
from uipc import Scene, SceneIO
from uipc.geometry import SimplicialComplex, SimplicialComplexIO

REPO_ID = 'MuGdxy/uipc-assets'

__all__ = ['REPO_ID', 'list_assets', 'asset_path', 'read_mesh', 'load', 'show']


def list_assets(*, revision: str = 'main') -> list[str]:
    """List all available asset names in the HuggingFace dataset.

    Returns:
        Sorted list of asset names (e.g. ``['cube_ground', 'fem_link_drop', ...]``).
    """
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
    name: str, *, revision: str = 'main', cache_dir: str | None = None
) -> pathlib.Path:
    """Download an asset by name from HuggingFace and return its local path.

    The result is cached by ``huggingface_hub``; subsequent calls with the same
    *name* and *revision* return instantly.

    Args:
        name: Asset name (e.g. ``'cube_ground'``).  Corresponds to
              ``assets/<name>/`` in the dataset repo.
        revision: Git revision (branch, tag, or commit hash).
        cache_dir: Where to cache downloaded files.  ``None`` uses the
                   default HuggingFace cache directory.

    Returns:
        :class:`~pathlib.Path` to the downloaded asset directory.
    """
    local_dir = snapshot_download(
        REPO_ID,
        allow_patterns=[f'assets/{name}/**'],
        revision=revision,
        cache_dir=cache_dir,
        repo_type='dataset',
    )
    result = pathlib.Path(local_dir) / 'assets' / name
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
    revision: str = 'main',
    cache_dir: str | None = None,
) -> None:
    """Download an asset and apply it to a Scene.

    Downloads ``assets/<name>/`` from the HuggingFace dataset, imports
    its ``scene.py``, and calls ``build_scene(scene)``.  Each asset's
    ``build_scene`` may modify ``scene.config()`` to set parameters
    like ``dt`` or ``contact/d_hat``.

    Args:
        name: Asset name (e.g. ``'cube_ground'``).
        scene: A :class:`uipc.Scene` instance to populate.
        revision: Git revision (branch, tag, or commit hash).
        cache_dir: Where to cache downloaded files.
    """
    path = asset_path(name, revision=revision, cache_dir=cache_dir)
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


def show(
    name: str,
    *,
    revision: str = 'main',
    cache_dir: str | None = None,
    distance_factor: float = 2.0,
) -> None:
    """Download an asset, build its scene, and display it in a Polyscope window.

    This is a convenience one-liner for quick visual inspection::

        from uipc.assets import show
        show('cube_ground')

    The camera is automatically positioned based on the scene bounding box:
    it looks at the center from a 45-degree elevation, pulled back by
    *distance_factor* times the bounding-box diagonal length.

    Args:
        name: Asset name (e.g. ``'cube_ground'``).
        revision: Git revision (branch, tag, or commit hash).
        cache_dir: Where to cache downloaded files.
        distance_factor: How far the camera sits relative to the bounding-box
                         diagonal (default ``2.0``).
    """
    import numpy as np
    import polyscope as ps
    from uipc import Engine, World
    from uipc.gui import SceneGUI

    scene = Scene(Scene.default_config())
    load(name, scene, revision=revision, cache_dir=cache_dir)

    engine = Engine('none')
    world = World(engine)
    world.init(scene)

    ps.init()
    gui = SceneGUI(scene)
    gui.register()
    gui.set_edge_width(1.0)

    _auto_camera(scene, distance_factor)
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
