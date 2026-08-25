"""
Regression test for FiniteElementExternalForce buffer drain.

Bug (pre-fix):
  FiniteElementExternalVertexForceConstraint::Impl::step() filtered the
  per-vertex is_constrained / external_force attributes into host arrays
  (`h_forces`, `h_vertex_ids`) and copied them to the device.  The copy was
  guarded by `if (!h_forces.empty())` — so when the user cleared all
  external forces (host arrays empty), the device-side buffers `forces`
  and `vertex_ids` retained their previous frame's contents.  Downstream,
  FiniteElementExternalVertexForce::do_step reads `forces.size()` from
  the *device* buffer; since the device size was never updated to 0, the
  scatter-add kernel re-applied stale forces on every advance() — forever.

Test:
  Single FEM tet (no gravity, no contact), run once through IPC and once through
  AL-IPC. The animator applies +x force on vertex 0 in frame 1 and clears it in
  frame 2. This verifies the shared clear -> animate -> consume lifecycle.
  Pass criterion: vertex_0.vx must DECREASE between frame 1 and frame 2.
  With the bug, the device buffer still holds (force=+x, vertex_id=0), so
  the +x force is re-applied → vx grows.

  The test also keeps timers disabled and rejects an unsolicited merged timing
  report. AL-IPC previously enabled the process-global timer and printed a
  report on every frame, leaking profiling state into later IPC worlds.
"""

import numpy as np
import pytest

from uipc import Engine, Scene, Timer, World, builtin, view
from uipc.constitution import ElasticModuli, FiniteElementExternalForce, StableNeoHookean
from uipc.core import FiniteElementStateAccessorFeature
from uipc.geometry import label_surface, label_triangle_orient, tetmesh

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


@pytest.mark.cuda
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("contact_constitution", ["ipc", "al-ipc"])
def test_finite_element_external_force_clear(
    contact_constitution, tmp_path, capfd, request
):
    Timer.disable_all()
    Timer.report_as_json()
    request.addfinalizer(Timer.disable_all)

    workspace = tmp_path / f"external_force_{contact_constitution}"
    engine = Engine("cuda", str(workspace))
    world = World(engine)

    config = Scene.default_config()
    config["gravity"] = [[0.0], [0.0], [0.0]]
    config["contact"]["enable"] = False
    config["contact"]["constitution"] = contact_constitution
    config["dt"] = 0.01

    scene = Scene(config)

    verts = np.array(
        [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
        dtype=np.float64,
    )
    tets = np.array([[0, 1, 2, 3]], dtype=np.int32)
    mesh = tetmesh(verts, tets)
    label_surface(mesh)
    label_triangle_orient(mesh)

    snk = StableNeoHookean()
    moduli = ElasticModuli.youngs_poisson(1e6, 0.4)
    snk.apply_to(mesh, moduli, mass_density=1000.0)

    ext = FiniteElementExternalForce()
    ext.apply_to(mesh, np.array([0.0, 0.0, 0.0], dtype=np.float64))

    obj = scene.objects().create("body")
    obj.geometries().create(mesh)

    def animate_force(info):
        geom = info.geo_slots()[0].geometry()
        force_attr = geom.vertices().find("external_force")
        is_constrained_attr = geom.vertices().find(builtin.is_constrained)
        assert force_attr is not None
        assert is_constrained_attr is not None

        force_view = view(force_attr)
        constrained_view = view(is_constrained_attr)
        force_view[:] = 0.0
        constrained_view[:] = 0
        if info.frame() == 1:
            constrained_view[0] = 1
            force_view[0] = np.array([[100.0], [0.0], [0.0]], dtype=np.float64)

    scene.animator().insert(obj, animate_force)

    world.init(scene)

    feat = world.features().find(FiniteElementStateAccessorFeature)
    assert feat is not None

    def read_vx_of_vertex0():
        sg = feat.create_geometry()
        sg.vertices().create(builtin.position, np.zeros(3, dtype=np.float64))
        sg.vertices().create(builtin.velocity, np.zeros(3, dtype=np.float64))
        feat.copy_to(sg)
        v = np.array(view(sg.vertices().find(builtin.velocity)))
        return float(v[0, 0, 0])  # vertex 0, x component

    # Frame 1: the animator applies +x force on vertex 0.
    world.advance()
    world.retrieve()
    vx_frame1 = read_vx_of_vertex0()

    # Frame 2: the animator clears all external forces.
    world.advance()
    world.retrieve()
    vx_frame2 = read_vx_of_vertex0()

    # With force cleared, vertex_0 should decelerate (elastic restoring).
    # With the bug, the device buffer still holds the +x force and the
    # scatter-add re-applies it → velocity grows.
    assert vx_frame1 > 0, f"frame 1 should accelerate vertex_0 in +x; got vx={vx_frame1}"
    assert vx_frame2 < vx_frame1, (
        f"after clearing force, vertex_0.vx should DECREASE; "
        f"got vx_frame1={vx_frame1:.4f}, vx_frame2={vx_frame2:.4f}. "
        f"If vx_frame2 > vx_frame1, the device-side force buffer was not drained."
    )

    captured = capfd.readouterr()
    assert "TIMING BREAKDOWN: MERGED TIMERS" not in captured.out

    frame_stats = engine.frame_stats()
    assert frame_stats["schema_version"] == 1
    assert frame_stats["pipeline"] == contact_constitution
    assert frame_stats["frame"] == 2
    assert frame_stats["completed"]
    assert frame_stats["newton_iterations"] >= 1
    assert frame_stats["line_search_trials"] >= 1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
