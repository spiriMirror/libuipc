"""Issue #499: force activation, soft-constraint independence, and buffer drain."""

import numpy as np
import pytest

from uipc import Engine, Scene, World, builtin, view
from uipc.constitution import (
    AffineBodyConstitution,
    AffineBodyExternalBodyForce,
    ElasticModuli2D,
    FiniteElementExternalForce,
    NeoHookeanShell,
    SoftPositionConstraint,
    SoftTransformConstraint,
)
from uipc.geometry import label_surface, label_triangle_orient, tetmesh, trimesh

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


FORCE_MASK = "external_force/is_constrained"
G = 9.81
DT = 0.01


def make_body(kind):
    if kind == "fem":
        mesh = trimesh(
            np.array([[0.0, 1.0, 0.0], [0.1, 1.0, 0.0], [0.0, 1.0, 0.1]]),
            np.array([[0, 1, 2]], dtype=np.int32),
        )
        rho, radius = 200.0, 0.001
        NeoHookeanShell().apply_to(
            mesh, ElasticModuli2D.youngs_poisson(1e5, 0.3), rho, radius
        )
        mass = 0.5 * 0.1 * 0.1 * rho * (2 * radius) / 3
    else:
        # Center the rest geometry so a translational force has no affine moment.
        vertices = np.array(
            [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]]
        )
        vertices -= vertices.mean(axis=0)
        mesh = tetmesh(vertices, np.array([[0, 1, 2, 3]], dtype=np.int32))
        AffineBodyConstitution().apply_to(mesh, 1e8, 1000.0)
        mass = 0.1**3 / 6 * 1000.0
    label_surface(mesh)
    if kind != "fem":
        label_triangle_orient(mesh)
    return mesh, np.array([0.0, -G * mass, 0.0])


def attributes(mesh, kind):
    return mesh.vertices() if kind == "fem" else mesh.instances()


def external_force(kind):
    return FiniteElementExternalForce() if kind == "fem" else AffineBodyExternalBodyForce()


def apply_soft_constraint(mesh, kind):
    if kind == "fem":
        SoftPositionConstraint().apply_to(mesh, 100.0)
    else:
        SoftTransformConstraint().apply_to(mesh, np.array([100.0, 100.0]))


@pytest.mark.parametrize("kind", ["fem", "abd3", "abd12"])
@pytest.mark.parametrize("soft_first", [False, True])
def test_external_force_enable_api(kind, soft_first):
    mesh, force = make_body(kind)
    if kind != "fem":
        mesh.instances().resize(2)
    if kind == "abd12":
        force = np.arange(1, 13, dtype=np.float64)
    attrs = attributes(mesh, kind)
    if soft_first:
        apply_soft_constraint(mesh, kind)
        view(attrs.find(builtin.is_constrained))[::2] = 1
        expected_soft_mask = np.array(view(attrs.find(builtin.is_constrained)))

    ext = external_force(kind)
    ext.apply_to(mesh, force)
    np.testing.assert_array_equal(view(attrs.find(FORCE_MASK)), 1)
    if soft_first:
        np.testing.assert_array_equal(
            view(attrs.find(builtin.is_constrained)), expected_soft_mask
        )
    if not soft_first:
        assert attrs.find(builtin.is_constrained) is None
        apply_soft_constraint(mesh, kind)
        np.testing.assert_array_equal(view(attrs.find(builtin.is_constrained)), 0)
        view(attrs.find(builtin.is_constrained))[::2] = 1
        np.testing.assert_array_equal(view(attrs.find(FORCE_MASK)), 1)
    soft_mask = np.array(view(attrs.find(builtin.is_constrained)))

    expected = force
    if kind == "abd3":
        expected = np.r_[force, np.zeros(9)]
    stored = np.array(view(attrs.find("external_force"))).reshape(-1, len(expected))
    np.testing.assert_allclose(stored, np.tile(expected, (len(stored), 1)))

    # Reapplying explicitly resets only the force mask, including mixed selections.
    view(attrs.find(FORCE_MASK))[::2] = 0
    for enable in [False, True]:
        ext.apply_to(mesh, force, enable=enable)
        np.testing.assert_array_equal(view(attrs.find(FORCE_MASK)), int(enable))
        np.testing.assert_array_equal(view(attrs.find(builtin.is_constrained)), soft_mask)


def make_world(tmp_path, mesh, pipeline, gravity=0.0):
    config = Scene.default_config()
    config["dt"] = DT
    config["integrator"]["type"] = "bdf1"
    config["gravity"] = [[0.0], [-gravity], [0.0]]
    config["contact"]["enable"] = False
    config["contact"]["constitution"] = pipeline
    scene = Scene(config)
    obj = scene.objects().create("body")
    slot, _ = obj.geometries().create(mesh)
    engine = Engine("cuda", str(tmp_path))
    # World holds a weak Engine reference; callers retain all three owners.
    return engine, World(engine), scene, obj, slot


def positions(slot, kind):
    mesh = slot.geometry()
    if kind == "fem":
        return np.array(mesh.positions().view()).reshape(-1, 3)
    return np.array(mesh.transforms().view())[:, :3, 3]


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("pipeline", ["ipc", "al-ipc"])
@pytest.mark.parametrize("kind", ["fem", "abd"])
@pytest.mark.parametrize("mode", ["force", "soft_first", "force_first", "legacy", "gravity"])
def test_external_force_matches_gravity(tmp_path, pipeline, kind, mode):
    mesh, force = make_body(kind)
    if mode == "soft_first":
        apply_soft_constraint(mesh, kind)
    if mode != "gravity":
        external_force(kind).apply_to(mesh, force)
    if mode == "force_first":
        apply_soft_constraint(mesh, kind)
    if mode == "legacy":
        attrs = attributes(mesh, kind)
        attrs.destroy(FORCE_MASK)
        attrs.create(builtin.is_constrained, 1)

    engine, world, scene, obj, slot = make_world(
        tmp_path, mesh, pipeline, gravity=G if mode == "gravity" else 0.0
    )
    world.init(scene)
    assert world.is_valid()
    start = positions(slot, kind)
    steps = 50
    for _ in range(steps):
        world.advance()
        world.retrieve()
        assert world.is_valid()
    expected = np.zeros_like(start)
    expected[:, 1] = -G * DT**2 * steps * (steps + 1) / 2
    # The gravity-only AL reference accumulates sub-micron lateral residuals
    # over 50 steps; allow one micron without requiring bitwise trajectories.
    np.testing.assert_allclose(positions(slot, kind) - start, expected, rtol=1e-5, atol=1e-6)


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("pipeline", ["ipc", "al-ipc"])
@pytest.mark.parametrize("kind", ["fem", "abd"])
@pytest.mark.parametrize("legacy", [False, True])
def test_external_force_toggle(tmp_path, pipeline, kind, legacy):
    mesh, force = make_body(kind)
    external_force(kind).apply_to(mesh, force, enable=False)
    attrs = attributes(mesh, kind)
    if legacy:
        attrs.destroy(FORCE_MASK)
    # In the new layout, a conflicting shared mask must not enable the force.
    attrs.create(builtin.is_constrained, 0 if legacy else 1)
    mask_name = builtin.is_constrained if legacy else FORCE_MASK
    engine, world, scene, obj, slot = make_world(tmp_path, mesh, pipeline)
    enabled = [0, 1, 1, 0, 0, 1]

    def animate(info):
        attrs = attributes(info.geo_slots()[0].geometry(), kind)
        view(attrs.find(mask_name))[:] = enabled[info.frame() - 1]
        # Leave nonzero force values intact: the mask alone must drain the buffers.

    scene.animator().insert(obj, animate)
    world.init(scene)
    assert world.is_valid()
    start = positions(slot, kind)
    velocity, displacement = 0.0, 0.0
    for active in enabled:
        velocity -= active * G * DT
        displacement += velocity * DT
        world.advance()
        world.retrieve()
        assert world.is_valid()
        expected = np.zeros_like(start)
        expected[:, 1] = displacement
        np.testing.assert_allclose(
            positions(slot, kind) - start, expected, rtol=1e-5, atol=1e-7
        )
