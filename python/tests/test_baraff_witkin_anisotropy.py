"""Baraff-Witkin API compatibility and directional CUDA response."""

import numpy as np
import pytest

from uipc import Engine, Scene, World, builtin, view
from uipc.constitution import ElasticModuli2D, StrainLimitingBaraffWitkinShell
from uipc.core import SceneIO
from uipc.geometry import label_surface, trimesh

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


def moduli(young, poisson=0.3):
    return ElasticModuli2D.youngs_poisson(young, poisson)


def patch():
    return trimesh(
        np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]),
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32),
    )


@pytest.mark.parametrize("legacy", ["default", "one_positional", "one_keyword", "two_positional", "two_keyword"])
def test_legacy_api_clears_independent_stretch(legacy):
    mesh = patch()
    shell = StrainLimitingBaraffWitkinShell()
    shell.apply_to(mesh, stretch_a_moduli=moduli(2e4),
                   stretch_b_moduli=moduli(8e3, 0.2), shear_moduli=moduli(10))
    assert mesh.triangles().find("stretch_stiffness_b") is not None
    if legacy == "default":
        shell.apply_to(mesh)
        young, poisson, shear = 1e6, 0.49, 1e6 / (2 * 1.49)
    else:
        young, poisson = 2e4, 0.3
        shear = 10 / 2.6 if legacy.startswith("two") else young / 2.6
        if legacy == "one_positional":
            shell.apply_to(mesh, moduli(young), 200., 0.001, 100.)
        elif legacy == "one_keyword":
            shell.apply_to(mesh, moduli=moduli(young))
        elif legacy == "two_positional":
            shell.apply_to(mesh, moduli(young), moduli(10), 200., 0.001, 100.)
        else:
            shell.apply_to(mesh, stretch_moduli=moduli(young), shear_moduli=moduli(10))
    assert mesh.triangles().find("stretch_stiffness_b") is None
    np.testing.assert_allclose(mesh.triangles().find("lambda").view(),
                               young * 0.002 / (1 - poisson**2))
    np.testing.assert_allclose(mesh.triangles().find("mu").view(), shear)


@pytest.mark.parametrize("independent", [False, True])
def test_directional_material_roundtrip(independent):
    mesh = patch()
    shell = StrainLimitingBaraffWitkinShell()
    if independent:
        shell.apply_to(mesh, stretch_a_moduli=moduli(2e4, 0.3),
                       stretch_b_moduli=moduli(8e3, 0.2), shear_moduli=moduli(10, 0.25),
                       thickness=0.002, strain_rate=40.)
        # Reapplication must replace manual edits on every triangle.
        view(mesh.triangles().find("stretch_stiffness_b"))[:] = -1
        shell.apply_to(mesh, stretch_a_moduli=moduli(2e4, 0.3),
                       stretch_b_moduli=moduli(8e3, 0.2), shear_moduli=moduli(10, 0.25),
                       thickness=0.002, strain_rate=40.)
    else:
        shell.apply_to(mesh, moduli(2e4, 0.3), moduli(10, 0.25),
                       thickness=0.002, strain_rate=40.)
    scene = Scene()
    obj = scene.objects().create("cloth")
    slot, _ = obj.geometries().create(mesh)
    loaded = SceneIO.from_json(SceneIO(scene).to_json())
    restored, _ = loaded.geometries().find(slot.id())
    attrs = restored.geometry().triangles()
    np.testing.assert_allclose(attrs.find("lambda").view(), 2e4 * 0.004 / (1 - 0.3**2))
    np.testing.assert_allclose(attrs.find("mu").view(), 4.)
    np.testing.assert_allclose(attrs.find("strain_rate").view(), 40.)
    b = attrs.find("stretch_stiffness_b")
    if independent:
        np.testing.assert_allclose(b.view(), 8e3 * 0.004 / (1 - 0.2**2))
    else:
        assert b is None


def test_directional_overload_requires_all_three_keywords():
    mesh = patch()
    shell = StrainLimitingBaraffWitkinShell()
    with pytest.raises(TypeError):
        shell.apply_to(mesh, stretch_a_moduli=moduli(2e4), stretch_b_moduli=moduli(8e3))
    with pytest.raises(TypeError):
        shell.apply_to(mesh, moduli(2e4), moduli(8e3), moduli(10))


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("axis", [0, 1])
def test_directional_cuda_response_and_legacy_fallback(tmp_path, axis):
    """Identical loading: the stiffer loaded axis stretches less.

    Shared-attribute edits, missing strain_rate in old scenes, explicit equal
    moduli, and reapplication all produce the same legacy trajectory.
    """
    config = Scene.default_config()
    config["dt"] = 0.01
    config["gravity"] = [[0.], [0.], [0.]]
    config["gravity"][axis][0] = 9.8
    config["contact"]["enable"] = False
    config["newton"]["velocity_tol"] = 1e-6
    scene = Scene(config)
    shell = StrainLimitingBaraffWitkinShell()
    slots = {}
    starts = {}
    soft, stiff, shear = moduli(2e3), moduli(2e5), moduli(10)
    for index, mode in enumerate(["legacy", "equal", "reapplied", "a_stiff", "b_stiff"]):
        mesh = patch()
        if mode == "legacy":
            shell.apply_to(mesh, stiff, shear)
            # Old data may omit strain_rate and directly edit shared lambda.
            mesh.triangles().destroy("strain_rate")
            view(mesh.triangles().find("lambda"))[:] = 2e3 * 0.002 / (1 - 0.3**2)
        else:
            shell.apply_to(mesh, stretch_a_moduli=stiff if mode == "a_stiff" else soft,
                           stretch_b_moduli=stiff if mode in ("b_stiff", "reapplied") else soft,
                           shear_moduli=shear)
            if mode == "reapplied":
                shell.apply_to(mesh, soft, shear)
        vertices = np.array(mesh.positions().view()).reshape(-1, 3)
        fixed = mesh.vertices().find(builtin.is_fixed)
        view(fixed)[vertices[:, axis] == 0] = 1
        view(mesh.positions())[:, 2] = 2. * index
        label_surface(mesh)
        obj = scene.objects().create(mode)
        slot, _ = obj.geometries().create(mesh)
        slots[mode] = slot
        starts[mode] = np.array(slot.geometry().positions().view()).reshape(-1, 3)
    engine = Engine("cuda", str(tmp_path))
    world = World(engine)
    world.init(scene)
    assert world.is_valid()
    for _ in range(40):
        world.advance()
        world.retrieve()
        assert world.is_valid()
    motions = {}
    for mode, slot in slots.items():
        result = np.array(slot.geometry().positions().view()).reshape(-1, 3)
        assert np.isfinite(result).all()
        motions[mode] = result - starts[mode]
        np.testing.assert_allclose(motions[mode][starts[mode][:, axis] == 0], 0, atol=1e-10)
    for mode in ("equal", "reapplied"):
        np.testing.assert_allclose(motions[mode], motions["legacy"],
                                   atol=2e-5, rtol=2e-4, err_msg=mode)
    loaded_stiff = "a_stiff" if axis == 0 else "b_stiff"
    assert motions["legacy"][:, axis].max() > 0.005
    assert motions[loaded_stiff][:, axis].max() < 0.5 * motions["legacy"][:, axis].max()
