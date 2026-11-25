"""Check contact exports after convergence, cubic rollback, and recovery."""

import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


def run_scene(workspace, case):
    from uipc import Engine, Scene, World, view
    from uipc.constitution import Particle
    from uipc.core import ContactSystemFeature
    from uipc.geometry import Geometry, ground, label_surface, pointcloud

    config = Scene.default_config()
    config["dt"] = 0.02
    config["gravity"] = [[0.0], [-9.8], [0.0]]
    config["contact"]["d_hat"] = 0.01
    config["newton"]["semi_implicit"]["enable"] = 0
    config["newton"]["velocity_tol"] = 1.0
    config["newton"]["velocity_tol_relative"] = 0.0
    config["line_search"]["report_energy"] = 0
    if case == "cubic":
        config["gravity"] = [[0.0], [-25.0], [0.0]]
        config["contact"]["adaptive"]["kappa_eval_scale"] = 3.1622776601683794e-12
        config["newton"]["max_iter"] = 1
        config["newton"]["velocity_tol"] = 0.001
        config["line_search"]["cubic_refinement"] = 1
    scene = Scene(config)
    scene.contact_tabular().default_model(0.0, 1e9)

    height = {"converged": 0.012, "recover": 0.04, "cubic": 0.015}[case]
    particle = pointcloud(np.array([[0.0, height, 0.0]]))
    label_surface(particle)
    Particle().apply_to(particle, 1e3, 0.001)
    scene.contact_tabular().default_element().apply_to(particle)
    obj = scene.objects().create("particle and ground")
    obj.geometries().create(particle)
    obj.geometries().create(ground(0))

    engine = Engine("cuda", workspace)
    world = World(engine)
    world.init(scene)
    feature = world.features().find(ContactSystemFeature)
    energy = Geometry()
    feature.contact_energy("PH+N", energy)
    assert energy.instances().size() == 0

    world.advance()
    world.retrieve()
    stats = engine.frame_stats()
    assert stats["completed"], stats

    if case == "recover":
        feature.contact_energy("PH+N", energy)
        assert energy.instances().size() == 0
        assert world.dump()
        for _ in range(3):
            world.advance()
            world.retrieve()
        exports = (
            (feature.contact_energy, energy),
            (feature.contact_gradient, Geometry()),
            (feature.contact_hessian, Geometry()),
        )
        for export, output in exports:
            export("PH+N", output)
            assert output.instances().size() == 1

        assert world.recover(1)
        world.retrieve()
        stats = engine.frame_stats()
        assert stats["frame"] == 1 and not stats["completed"], stats
        assert stats["newton_iterations"] == 0, stats
        assert stats["linear_solver_iterations"] == 0, stats
        assert stats["line_search_trials"] == 0, stats
        for export, output in exports:
            export("PH+N", output)
            assert output.instances().size() == 0

        for _ in range(3):
            world.advance()
            world.retrieve()
        assert engine.frame_stats()["completed"]
        feature.contact_energy("PH+N", energy)
        assert energy.instances().size() == 1
        return

    if case == "cubic":
        # The rejected interpolation trial drops the contact. The accepted
        # half step restores it on the last permitted Newton iteration.
        assert stats["hit_newton_limit"], stats
        assert stats["line_search_trials"] == 3, stats
        assert stats["last_line_search_alpha"] == 0.5, stats
    else:
        assert stats["converged"], stats
    assert stats["newton_iterations"] == 1, stats
    feature.contact_energy("PH+N", energy)
    assert energy.instances().size() == 1
    values = view(energy.instances().find("energy"))
    assert np.isfinite(values).all() and (values > 0).all()


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.cuda
@pytest.mark.basic
@pytest.mark.parametrize("case", ["converged", "recover", "cubic"])
def test_contact_energy_export(tmp_path, case):
    tests_dir = str(Path(__file__).resolve().parents[1])
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        path for path in (tests_dir, env.get("PYTHONPATH")) if path
    )
    result = subprocess.run(
        [sys.executable, __file__, str(tmp_path), case],
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
    )
    assert result.returncode == 0, result.stdout[-4000:] + result.stderr[-4000:]


if __name__ == "__main__":
    run_scene(sys.argv[1], sys.argv[2])
