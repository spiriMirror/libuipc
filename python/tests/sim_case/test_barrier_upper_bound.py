"""Regression: contact pairs sitting exactly on the barrier's upper bound.

Two affine-body plates are placed so that their facing faces are exactly
d_hat apart (gravity off, generic tilted pose), so many point-triangle pairs
have d^2 == d_hat^2 up to rounding. The collision filter and the energy kernel
evaluate d^2 independently; before the fix a 1-ulp disagreement tripped the
energy kernel's `is_active_D` assertion and aborted the process. The scene is
run in a subprocess so that an abort fails this test instead of pytest.
"""
import os
import subprocess
import sys

import numpy as np
import pytest

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


def plate(n, size, z0, z1):
    """Closed box whose top/bottom faces are n x n quad grids."""
    xs = np.linspace(-size / 2, size / 2, n + 1)
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    V = np.vstack([np.c_[X.ravel(), Y.ravel(), np.full(X.size, z)]
                   for z in (z0, z1)])
    m = (n + 1) ** 2

    def vid(i, j):
        return i * (n + 1) + j

    F = []
    for i in range(n):
        for j in range(n):
            a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
            F += [[a, c, b], [a, d, c], [m + a, m + b, m + c], [m + a, m + c, m + d]]
    rim = ([vid(i, 0) for i in range(n)] + [vid(n, j) for j in range(n)]
           + [vid(i, n) for i in range(n, 0, -1)] + [vid(0, j) for j in range(n, 0, -1)])
    for a, b in zip(rim, rim[1:] + rim[:1]):
        F += [[a, b, m + b], [a, m + b, m + a]]
    return V, np.asarray(F, dtype=np.int32)


def rotation(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def run_scene(workspace, frames=20):
    from uipc import Engine, World, Scene, view, builtin
    from uipc.geometry import trimesh, label_surface
    from uipc.constitution import AffineBodyConstitution

    d_hat = 5e-5
    R = rotation([1, 2, 3], 0.3)
    offset = np.array([0.0123, 0.0456, 0.0042])

    config = Scene.default_config()
    config["dt"] = 1e-3
    config["gravity"] = [[0.0], [0.0], [0.0]]
    config["contact"]["d_hat"] = d_hat
    scene = Scene(config)
    scene.contact_tabular().default_model(0.0, 1e9)
    abd = AffineBodyConstitution()

    # lower plate top face at z = 0, upper plate bottom face at z = d_hat
    for name, n, z0, z1, fixed in [("lower", 16, -0.01, 0.0, 1),
                                   ("upper", 17, d_hat, d_hat + 0.01, 0)]:
        V, F = plate(n, 0.1, z0, z1)
        sc = trimesh(V @ R.T + offset, F)
        label_surface(sc)
        abd.apply_to(sc, 1e8)
        scene.contact_tabular().default_element().apply_to(sc)
        view(sc.instances().find(builtin.is_fixed))[0] = fixed
        scene.objects().create(name).geometries().create(sc)

    engine = Engine("cuda", workspace)
    world = World(engine)
    world.init(scene)
    for _ in range(frames):
        world.advance()
        world.retrieve()


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.cuda
@pytest.mark.basic
def test_barrier_upper_bound(tmp_path):
    tests_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in (tests_dir, env.get("PYTHONPATH")) if p)
    proc = subprocess.run([sys.executable, __file__, str(tmp_path)], env=env,
                          capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, proc.stdout[-4000:] + proc.stderr[-4000:]


if __name__ == "__main__":
    run_scene(sys.argv[1])
