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
  Single FEM tet (no gravity, no contact).
  Frame 1: apply +x force on vertex 0, advance → vertex_0.vx > 0.
  Frame 2: clear all external forces (is_constrained[:]=0), advance.
  Pass criterion: vertex_0.vx must DECREASE between frame 1 and frame 2.
  With the bug, the device buffer still holds (force=+x, vertex_id=0), so
  the +x force is re-applied → vx grows.
"""

import numpy as np
import pytest

from uipc import Engine, World, Scene, view, builtin
from uipc.constitution import ElasticModuli, FiniteElementExternalForce, StableNeoHookean
from uipc.core import FiniteElementStateAccessorFeature
from uipc.geometry import label_surface, label_triangle_orient, tetmesh

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason


@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
def test_finite_element_external_force_clear():
    engine = Engine("cuda")
    world = World(engine)

    config = Scene.default_config()
    config["gravity"] = [[0.0], [0.0], [0.0]]
    config["contact"]["enable"] = False
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
    geom_slot, _ = obj.geometries().create(mesh)

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

    geom = geom_slot.geometry()
    force_attr = geom.vertices().find("external_force")
    is_constrained_attr = geom.vertices().find(builtin.is_constrained)
    assert force_attr is not None
    assert is_constrained_attr is not None
    fv = view(force_attr)
    cv = view(is_constrained_attr)

    # Frame 1: apply +x force on vertex 0
    cv[:] = 0
    cv[0] = 1
    fv[:] = 0.0
    fv[0] = np.array([[100.0], [0.0], [0.0]], dtype=np.float64)
    world.advance()
    world.retrieve()
    vx_frame1 = read_vx_of_vertex0()

    # Frame 2: CLEAR force, advance
    cv[:] = 0
    fv[:] = 0.0
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


if __name__ == "__main__":
    test_finite_element_external_force_clear()
    print("OK")
