"""CUDA checks for integrator history, Newton limits and restart consistency."""
import numpy as np
import pytest

from uipc import Engine, Scene, World, builtin, view
from uipc.constitution import AffineBodyConstitution, Particle
from uipc.geometry import ground, label_surface, pointcloud, tetmesh

pytestmark = pytest.mark.cuda


def make_world(path, config, vx=0.0, abd=False, floor=False):
    scene = Scene(config)
    scene.contact_tabular().default_model(0.8, 1e9)
    obj = scene.objects().create("body")
    if abd:
        mesh = tetmesh(np.vstack((np.zeros(3), np.eye(3))), np.array([[0, 1, 2, 3]], dtype=np.int32))
        AffineBodyConstitution().apply_to(mesh, 1e6, 1000.0)
        view(mesh.instances().find(builtin.velocity))[0, 0, 3] = vx
    else:
        mesh = pointcloud(np.array([[0.0, 0.03, 0.0]]))
        Particle().apply_to(mesh, 1000.0, 0.001)
        view(mesh.vertices().find(builtin.velocity))[0, 0, 0] = vx
    label_surface(mesh)
    scene.contact_tabular().default_element().apply_to(mesh)
    slot, _ = obj.geometries().create(mesh)
    if floor:
        obj.geometries().create(ground(0.0))
    engine = Engine("cuda", str(path))
    world = World(engine)
    world.init(scene)
    assert world.is_valid()
    return scene, engine, world, slot


@pytest.mark.parametrize("abd", [False, True], ids=["fem", "abd"])
def test_bdf2_preserves_initial_velocity(tmp_path, abd):
    cfg = Scene.default_config()
    cfg["dt"] = 0.03
    cfg["gravity"] = [[0.0], [-9.8], [0.0]]
    cfg["integrator"]["type"] = "bdf2"
    scene, engine, world, slot = make_world(tmp_path, cfg, vx=1.0, abd=abd)
    for i in range(1, 4):
        world.advance()
        world.retrieve()
        assert world.is_valid()
        geom = slot.geometry()
        position = geom.transforms().view()[0, :3, 3] if abd else geom.positions().view()[0, :, 0]
        assert position[0] == pytest.approx(i * cfg["dt"], abs=1e-8)
        if i == 1:
            assert world.dump()
    expected = position.copy()
    assert world.recover(1)
    world.advance()
    world.advance()
    world.retrieve()
    actual = geom.transforms().view()[0, :3, 3] if abd else geom.positions().view()[0, :, 0]
    np.testing.assert_allclose(actual, expected, rtol=1e-7, atol=1e-9)


def test_newton_minimum_equals_maximum(tmp_path):
    cfg = Scene.default_config()
    cfg["gravity"] = [[0.0], [0.0], [0.0]]
    cfg["newton"].update(min_iter=2, max_iter=2)
    cfg["newton"]["semi_implicit"]["enable"] = 0
    scene, engine, world, slot = make_world(tmp_path, cfg)
    world.advance()
    stats = engine.frame_stats()
    assert stats["newton_iterations"] == 2
    assert stats["converged"] and not stats["hit_newton_limit"]


@pytest.mark.parametrize("pipeline", ["ipc", "al-ipc"])
def test_strict_limit_reaches_engine_status(tmp_path, pipeline):
    cfg = Scene.default_config()
    cfg["contact"]["constitution"] = pipeline
    cfg["extras"]["strict_mode"]["enable"] = 1
    cfg["newton"]["max_iter"] = 1
    cfg["newton"]["velocity_tol"] = 1e-12
    scene, engine, world, slot = make_world(tmp_path, cfg, vx=1.0, floor=True)
    world.advance()
    assert engine.status().has_error()
    assert not world.is_valid()
    assert not engine.frame_stats()["completed"]


def test_al_sliding_restart_restores_lagged_friction(tmp_path):
    cfg = Scene.default_config()
    cfg["dt"] = 0.02
    cfg["contact"].update(constitution="al-ipc", d_hat=0.01)
    scene, engine, world, slot = make_world(tmp_path, cfg, vx=1.5, floor=True)
    for _ in range(4):
        world.advance()
    assert world.dump()
    world.advance()
    world.retrieve()
    expected = slot.geometry().positions().view().copy()

    scene2, engine2, world2, slot2 = make_world(tmp_path, cfg, vx=1.5, floor=True)
    assert world2.recover(4)
    world2.advance()
    world2.retrieve()
    np.testing.assert_allclose(slot2.geometry().positions().view(), expected, rtol=1e-7, atol=1e-9)

    payload = next(tmp_path.rglob("PH_lambda.4"))
    data = payload.read_bytes()
    assert len(data) > 16
    payload.write_bytes(data[:-1])
    assert not world2.recover(4)
    assert world2.frame() == 5
    payload.unlink()
    assert not world2.recover(4)

    # A failed subsystem write must not publish a checkpoint for frame five.
    payload.with_name("PH_lambda.5").mkdir()
    assert not world2.dump()
    assert not list(tmp_path.rglob("state.5.json"))


def test_al_empty_history_checkpoint(tmp_path):
    cfg = Scene.default_config()
    cfg["contact"]["constitution"] = "al-ipc"
    cfg["gravity"] = [[0.0], [0.0], [0.0]]
    scene, engine, world, slot = make_world(tmp_path, cfg, vx=1.0)
    for frame in (0, 1):
        assert world.dump()
        world.advance()
        assert world.recover(frame)
        assert world.frame() == frame
        world.advance()
        assert world.is_valid()
