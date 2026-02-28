"""Tests for the uipc.profile module.

Verifies the benchmarking session API and WorldVisitor.engine()
using the 'none' backend (no CUDA required), plus CUDA integration tests.
"""
import pytest
import numpy as np

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason
from uipc import Engine, World, Scene, SceneIO
from uipc.backend import WorldVisitor
from uipc.geometry import (
    tetmesh, ground, pointcloud,
    label_surface, label_triangle_orient, flip_inward_triangles,
)
from uipc.profile import session, run


def _make_scene():
    """Create a minimal scene with a tet and ground."""
    scene = Scene(Scene.default_config())
    Vs = np.array([
        [0, 1, 0],
        [0, 0, 1],
        [-np.sqrt(3) / 2, 0, -0.5],
        [np.sqrt(3) / 2, 0, -0.5],
    ], dtype=np.float64)
    Ts = np.array([[0, 1, 2, 3]])
    tet = tetmesh(Vs, Ts)
    obj = scene.objects().create("obj")
    obj.geometries().create(tet)
    obj.geometries().create(ground())
    return scene


def _make_world(scene, workspace):
    """Create an Engine + World from a Scene using the 'none' backend."""
    engine = Engine('none', str(workspace))
    world = World(engine)
    world.init(scene)
    return engine, world


# ---------------------------------------------------------------------------
# WorldVisitor.engine()
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_world_visitor_engine(tmp_path):
    """WorldVisitor.engine() returns an Engine with the correct workspace."""
    scene = _make_scene()
    ws = tmp_path / 'test_ws_visitor'
    engine = Engine('none', str(ws))
    world = World(engine)
    world.init(scene)

    wv = WorldVisitor(world)
    recovered_engine = wv.engine()
    assert str(recovered_engine.workspace()) == str(engine.workspace())
    assert str(recovered_engine.backend_name()) == 'none'


@pytest.mark.basic
def test_world_visitor_scene_get(tmp_path):
    """WorldVisitor.scene().get() returns a usable Scene."""
    scene = _make_scene()
    engine = Engine('none', str(tmp_path / 'ws_scene_get'))
    world = World(engine)
    world.init(scene)

    wv = WorldVisitor(world)
    extracted_scene = wv.scene().get()
    # The extracted scene should have the same objects
    assert extracted_scene.objects().size() > 0


# ---------------------------------------------------------------------------
# Engine + World setup
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_make_world(tmp_path):
    """Engine + World can be created from a Scene."""
    scene = _make_scene()
    engine, world = _make_world(scene, tmp_path / 'ws_make_world')
    assert world.is_valid()
    assert world.frame() == 0


# ---------------------------------------------------------------------------
# Session with Scene (shortcut)
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_session_scene_profile():
    """session(scene) creates a world internally and profiles frames."""
    scene = _make_scene()
    with session(scene, name='test', backend='none') as s:
        s.profile(3)

    assert s.result is not None
    assert s.result['num_frames'] == 3
    assert s.result['wall_time'] > 0
    assert s.result['name'] == 'test'
    assert len(s.result['timer_frames']) == 3


@pytest.mark.basic
def test_session_scene_advance_profile():
    """session(scene) supports advance + profile."""
    scene = _make_scene()
    with session(scene, name='test', backend='none') as s:
        s.advance(2)
        s.profile(3)

    assert s.result is not None
    assert s.result['num_frames'] == 3  # only profiled frames count
    assert s.result['steps'] == [('advance', 2), ('profile', 3)]


# ---------------------------------------------------------------------------
# Session with World (primary API)
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_session_world_direct(tmp_path):
    """session(world) uses the world directly without replay."""
    scene = _make_scene()
    engine, world = _make_world(scene, tmp_path / 'ws_world_direct')

    # Advance to frame 2
    for _ in range(2):
        world.advance()
        world.retrieve()
    assert world.frame() == 2

    with session(world, name='test') as s:
        s.profile(3)

    assert s.result is not None
    assert s.result['num_frames'] == 3
    # World should now be at frame 5
    assert world.frame() == 5


@pytest.mark.basic
def test_session_world_advance_profile(tmp_path):
    """session(world) supports advance + profile on the existing world."""
    scene = _make_scene()
    engine, world = _make_world(scene, tmp_path / 'ws_world_adv_prof')

    with session(world, name='test') as s:
        s.advance(2)
        s.profile(3)

    assert s.result is not None
    assert s.result['num_frames'] == 3
    assert world.frame() == 5


# ---------------------------------------------------------------------------
# run() simple API
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_run_world(tmp_path):
    """run(world) returns a result dict."""
    scene = _make_scene()
    engine, world = _make_world(scene, tmp_path / 'ws_run_world')

    result = run(world, num_frames=3, name='test')
    assert result['num_frames'] == 3
    assert result['wall_time'] > 0
    assert 'summary' in result


@pytest.mark.basic
def test_run_scene_shortcut():
    """run(scene) works as a convenience shortcut."""
    scene = _make_scene()

    result = run(scene, num_frames=2, name='test', backend='none')
    assert result['num_frames'] == 2


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_session_no_steps_raises():
    """session() raises if no steps are queued."""
    scene = _make_scene()
    with pytest.raises(ValueError, match='No steps queued'):
        with session(scene, name='test', backend='none') as s:
            pass  # no advance or profile calls


@pytest.mark.basic
def test_session_only_advance_raises():
    """session() raises if only advance (no profile) is queued."""
    scene = _make_scene()
    with pytest.raises(ValueError, match='No profile'):
        with session(scene, name='test', backend='none') as s:
            s.advance(5)


# ---------------------------------------------------------------------------
# CUDA integration tests
# ---------------------------------------------------------------------------

def _make_particle_scene():
    """Create a particle-ground scene suitable for CUDA simulation."""
    from uipc.constitution import Particle

    scene = Scene(Scene.default_config())
    pt = Particle()

    scene.contact_tabular().default_model(0.5, 1e9)
    default_element = scene.contact_tabular().default_element()

    n = 5
    Vs = np.zeros((n, 3), dtype=np.float64)
    for i in range(n):
        Vs[i][1] = i * 0.05 + 0.2

    mesh = pointcloud(Vs)
    label_surface(mesh)
    pt.apply_to(mesh, 1e3, 0.01)
    default_element.apply_to(mesh)

    obj = scene.objects().create("particles")
    obj.geometries().create(mesh)
    obj.geometries().create(ground(0.0))

    return scene


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_cuda_session_world(tmp_path):
    """CUDA: session(world) benchmarks from the world's current frame."""
    scene = _make_particle_scene()
    engine = Engine('cuda', str(tmp_path / 'ws_cuda_session'))
    world = World(engine)
    world.init(scene)

    # Advance 3 frames
    for _ in range(3):
        world.advance()
        world.retrieve()
    assert world.frame() == 3

    # Profile 2 more frames
    with session(world, name='cuda_test') as s:
        s.profile(2)

    assert s.result is not None
    assert s.result['num_frames'] == 2
    assert s.result['wall_time'] > 0
    assert world.frame() == 5
    assert len(s.result['timer_frames']) == 2


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_cuda_session_advance_profile(tmp_path):
    """CUDA: session(world) advance + profile with real simulation."""
    scene = _make_particle_scene()
    engine = Engine('cuda', str(tmp_path / 'ws_cuda_adv_prof'))
    world = World(engine)
    world.init(scene)

    with session(world, name='cuda_adv_prof') as s:
        s.advance(2)
        s.profile(3)

    assert s.result is not None
    assert s.result['num_frames'] == 3
    assert world.frame() == 5
    # Timer tree should have real data from the CUDA backend
    for frame_data in s.result['timer_frames']:
        assert 'children' in frame_data or 'name' in frame_data


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_cuda_run_world(tmp_path):
    """CUDA: run(world) returns a result with timer stats."""
    scene = _make_particle_scene()
    engine = Engine('cuda', str(tmp_path / 'ws_cuda_run'))
    world = World(engine)
    world.init(scene)

    result = run(world, num_frames=3, name='cuda_run')
    assert result['num_frames'] == 3
    assert result['wall_time'] > 0
    assert 'summary' in result
    assert len(result['timer_frames']) == 3


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_cuda_world_visitor_engine(tmp_path):
    """CUDA: WorldVisitor.engine() returns correct workspace/backend."""
    scene = _make_particle_scene()
    ws = tmp_path / 'test_cuda_ws'
    engine = Engine('cuda', str(ws))
    world = World(engine)
    world.init(scene)

    wv = WorldVisitor(world)
    recovered = wv.engine()
    assert str(recovered.workspace()).endswith('test_cuda_ws')
    assert str(recovered.backend_name()) == 'cuda'
