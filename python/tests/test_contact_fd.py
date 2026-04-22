"""Finite-difference derivative tests for DistanceDiagnoserFeature.

Tests the analytical gradient and Hessian of:
  * dist2    — squared distance
  * barrier  — IPC contact barrier energy
  * e_k      — edge-edge mollifier (EE only)

for all 4 contact primitive types (PP, PE, PT, EE) at positions spanning
every Voronoi (closest-feature) region, including positions exactly on the
region boundaries.

Derivative testing uses the Schroeder central-difference + central-average
formula (SIGGRAPH '22 Course "Practical course on computing derivatives in
code", Section 4.1):

    |z(x+Δx) − z(x−Δx) − (∇z(x+Δx) + ∇z(x−Δx)) · Δx| / δ  =  O(δ²)

With δ=1e-6 the normalised error is O(δ²)=O(1e-12)/δ = O(1e-6), well below
the pass threshold of 1e-4.  A wrong derivative gives O(1) error.

Notes on Voronoi boundaries
---------------------------
dist2 is C¹ (gradient is continuous) but NOT C² (Hessian is discontinuous)
at Voronoi region boundaries.  For positions EXACTLY on a boundary:
  - gradient test:  still valid, still run
  - Hessian test:   deliberately skipped (Hessian is not uniquely defined there)
The ``test_hess=False`` flag is set for those boundary cases.

References to ipc-toolkit
-------------------------
Each test docstring below cites the corresponding ipc-toolkit Catch2 tests
under ``ipc-toolkit/tests/src/tests/distance/``.  ipc-toolkit performs the
same kind of FD checks via the ``finitediff`` library (zfergus/finite-diff),
whose ``fd::compare_gradient`` and ``fd::compare_hessian`` use a default
``test_eps = 1e-4`` -- matching our ``_GRAD_THRESH`` / ``_HESS_THRESH``.
The C¹-only handling at Voronoi boundaries (dist2) and at the mollifier
threshold ``x = eps_x`` (e_k) also follows ipc-toolkit's conventions.
"""

import numpy as np
import pytest

from conftest import skip_cuda_on_macos, skip_cuda_on_macos_reason
from uipc import Logger, view
from uipc.core import Engine, World, Scene, DistanceDiagnoserFeature
from uipc.geometry import Geometry, pointcloud, linemesh, trimesh

Logger.set_level(Logger.Level.Warn)

# ── tuning constants ──────────────────────────────────────────────────────────
_DELTA       = 1e-6     # Schroeder perturbation scale
_GRAD_THRESH = 1e-4     # normalised-error pass threshold (gradient tests)
_HESS_THRESH = 1e-4     # normalised-error pass threshold (Hessian tests)
_EK_THRESH   = 1e-3     # looser threshold for mollifier quantities
_D_HAT       = 0.5      # barrier activation distance applied to all vertices
_THICKNESS   = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Session fixture
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="session")
def ddf_session(tmp_path_factory):
    """CUDA engine + DDF feature shared across all tests in the session.

    Skips the entire session gracefully if CUDA / the CUDA backend is absent.
    The engine reference is kept alive to prevent premature GC.
    """
    try:
        workspace = str(tmp_path_factory.mktemp("contact_fd"))
        engine = Engine("cuda", workspace)
        world  = World(engine)
        scene  = Scene(Scene.default_config())
        world.init(scene)
        ddf = world.features().find(DistanceDiagnoserFeature)
        assert ddf is not None, "DistanceDiagnoserFeature not found in world features"
    except Exception as exc:
        pytest.skip(f"CUDA / DDF unavailable: {exc}")
    yield engine, world, ddf  # keep engine alive to prevent GC


# ─────────────────────────────────────────────────────────────────────────────
# Schroeder derivative-test helpers  (course Section 4.1)
# ─────────────────────────────────────────────────────────────────────────────

def _n_dof(geom_list):
    """Total number of position DOFs across all geometries in the list."""
    return sum(view(sc.positions()).shape[0] * 3 for sc in geom_list)


def _apply_perturbation(geom_list, originals, dx_flat):
    """Write  orig + dx_flat[offset:offset+3*n_v]  into each geometry.

    ``originals`` must be a list of .copy() snapshots taken before any
    perturbation; this ensures successive calls do not compound.
    ``dx_flat`` is a concatenated (3*n_total,) float64 array.
    """
    offset = 0
    for sc, orig in zip(geom_list, originals):
        n_v  = orig.shape[0]
        pos  = view(sc.positions())
        pos[:] = orig + dx_flat[offset:offset + 3 * n_v].reshape(orig.shape)
        offset += 3 * n_v


def schroeder_grad_test(compute_fn, geom_list, get_scalar_fn, get_grad_fn,
                        delta=_DELTA, seed=42):
    """Schroeder gradient test (Section 4.1).

    Applies one random perturbation Δx ~ Uniform(−δ, δ)^n and checks:

        |z(x+Δx) − z(x−Δx) − (∇z(x+Δx) + ∇z(x−Δx)) · Δx| / δ = O(δ²)

    Returns the normalised error, or None when any attribute is unavailable.
    """
    rng = np.random.default_rng(seed)
    n   = _n_dof(geom_list)
    dx  = rng.uniform(-delta, delta, n)
    originals = [view(sc.positions()).copy() for sc in geom_list]

    _apply_perturbation(geom_list, originals, +dx)
    compute_fn()
    f_plus = get_scalar_fn()
    g_plus = get_grad_fn()

    _apply_perturbation(geom_list, originals, -dx)
    compute_fn()
    f_minus = get_scalar_fn()
    g_minus = get_grad_fn()

    _apply_perturbation(geom_list, originals, np.zeros(n))
    compute_fn()

    if f_plus is None or f_minus is None or g_plus is None or g_minus is None:
        return None

    g_plus  = np.asarray(g_plus,  dtype=np.float64)
    g_minus = np.asarray(g_minus, dtype=np.float64)

    f_diff   = float(f_plus) - float(f_minus)
    g_dot_dx = float(np.dot(g_plus + g_minus, dx))
    normed   = abs(f_diff - g_dot_dx) / delta

    print(f"\n    grad  |f_diff|={abs(f_diff):.3e}  "
          f"|g*dx|={abs(g_dot_dx):.3e}  err/delta={normed:.3e}")
    return normed


def schroeder_hess_test(compute_fn, geom_list, get_grad_fn, get_hess_fn,
                        delta=_DELTA, seed=42):
    """Schroeder Hessian test.  z = gradient, ∇z = Hessian.

    Checks:
        ||∇z(x+Δx) − ∇z(x−Δx) − (H(x+Δx) + H(x−Δx)) Δx|| / δ = O(δ²)

    Returns the normalised error, or None when any attribute is unavailable.
    """
    rng = np.random.default_rng(seed)
    n   = _n_dof(geom_list)
    dx  = rng.uniform(-delta, delta, n)
    originals = [view(sc.positions()).copy() for sc in geom_list]

    _apply_perturbation(geom_list, originals, +dx)
    compute_fn()
    g_plus = get_grad_fn()
    H_plus = get_hess_fn()

    _apply_perturbation(geom_list, originals, -dx)
    compute_fn()
    g_minus = get_grad_fn()
    H_minus = get_hess_fn()

    _apply_perturbation(geom_list, originals, np.zeros(n))
    compute_fn()

    if g_plus is None or g_minus is None or H_plus is None or H_minus is None:
        return None

    g_plus  = np.asarray(g_plus,  dtype=np.float64)
    g_minus = np.asarray(g_minus, dtype=np.float64)
    H_plus  = np.asarray(H_plus,  dtype=np.float64).reshape(len(g_plus), -1)
    H_minus = np.asarray(H_minus, dtype=np.float64).reshape(len(g_minus), -1)

    residual = (g_plus - g_minus) - (H_plus + H_minus) @ dx[:len(g_plus)]
    normed   = np.linalg.norm(residual) / delta

    print(f"\n    hess  |g_diff|={np.linalg.norm(g_plus - g_minus):.3e}  "
          f"|H*dx|={np.linalg.norm((H_plus+H_minus)@dx[:len(g_plus)]):.3e}  "
          f"err/delta={normed:.3e}")
    return normed


# ─────────────────────────────────────────────────────────────────────────────
# Attribute readers
# ─────────────────────────────────────────────────────────────────────────────

def _scalar(R, name):
    attr = R.instances().find(name)
    return None if attr is None else float(attr.view().flatten()[0])


def _vec(R, name, n):
    attr = R.instances().find(name)
    return None if attr is None else attr.view().flatten()[:n].astype(np.float64)


def _mat(R, name, n):
    attr = R.instances().find(name)
    return None if attr is None else (
        attr.view().flatten()[:n * n].astype(np.float64).reshape(n, n)
    )


# ─────────────────────────────────────────────────────────────────────────────
# Geometry builders
# ─────────────────────────────────────────────────────────────────────────────

def _pc(verts):
    """Pointcloud geometry with d_hat and thickness attributes."""
    sc = pointcloud(np.asarray(verts, dtype=np.float64))
    sc.vertices().create("d_hat",     _D_HAT)
    sc.vertices().create("thickness", _THICKNESS)
    return sc


def _lm(verts):
    """Linemesh geometry with d_hat and thickness attributes."""
    sc = linemesh(
        np.asarray(verts, dtype=np.float64),
        np.array([[0, 1]], dtype=np.int32),
    )
    sc.vertices().create("d_hat",     _D_HAT)
    sc.vertices().create("thickness", _THICKNESS)
    return sc


def _lm_rest(verts):
    """Linemesh for rest positions — no d_hat / thickness required."""
    return linemesh(
        np.asarray(verts, dtype=np.float64),
        np.array([[0, 1]], dtype=np.int32),
    )


def _tm(verts):
    """Trimesh geometry with d_hat and thickness attributes."""
    sc = trimesh(
        np.asarray(verts, dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int32),
    )
    sc.vertices().create("d_hat",     _D_HAT)
    sc.vertices().create("thickness", _THICKNESS)
    return sc


# ─────────────────────────────────────────────────────────────────────────────
# Test helper: run the full FD battery for one distance/barrier pair
# ─────────────────────────────────────────────────────────────────────────────

def _check_dist2_barrier(compute, geom_list, R, n, test_hess, label):
    """Run gradient (always) and Hessian (if test_hess) checks on dist2 and
    barrier for the current geometry configuration."""

    # ── dist2 gradient ───────────────────────────────────────────────────────
    err = schroeder_grad_test(
        compute, geom_list,
        lambda: _scalar(R, "dist2"),
        lambda: _vec(R, "dist2/grad", n),
        seed=0,
    )
    assert err is not None and err < _GRAD_THRESH, (
        f"[{label}] dist2 grad normalised-error={err:.3e} >= {_GRAD_THRESH}"
    )

    # ── dist2 Hessian ────────────────────────────────────────────────────────
    if test_hess:
        err = schroeder_hess_test(
            compute, geom_list,
            lambda: _vec(R, "dist2/grad", n),
            lambda: _mat(R, "dist2/hess", n),
            seed=1,
        )
        assert err is not None and err < _HESS_THRESH, (
            f"[{label}] dist2 hess normalised-error={err:.3e} >= {_HESS_THRESH}"
        )

    # ── barrier gradient (only when barrier is active) ───────────────────────
    b_val = _scalar(R, "barrier")
    assert b_val is not None, f"[{label}] barrier attribute missing"
    if b_val > 0.0:

        err = schroeder_grad_test(
            compute, geom_list,
            lambda: _scalar(R, "barrier"),
            lambda: _vec(R, "barrier/grad", n),
            seed=2,
        )
        assert err is not None and err < _GRAD_THRESH, (
            f"[{label}] barrier grad normalised-error={err:.3e} >= {_GRAD_THRESH}"
        )

        # ── barrier Hessian ──────────────────────────────────────────────────
        if test_hess:
            err = schroeder_hess_test(
                compute, geom_list,
                lambda: _vec(R, "barrier/grad", n),
                lambda: _mat(R, "barrier/hess", n),
                seed=3,
            )
            assert err is not None and err < _HESS_THRESH, (
                f"[{label}] barrier hess normalised-error={err:.3e} >= {_HESS_THRESH}"
            )


# ─────────────────────────────────────────────────────────────────────────────
# ═══════════════════════════════  POINT-POINT  ═══════════════════════════════
# ─────────────────────────────────────────────────────────────────────────────
# Point-point has exactly one Voronoi region (P_P); all cases test grad + hess.

_PP_CASES = [
    # (p0, p1, label)
    ([0.0, 0.0, 0.0], [0.0, 0.20, 0.0],   "along_y"),
    ([0.0, 0.0, 0.0], [0.10, 0.15, 0.10], "diagonal"),
    ([0.1, 0.1, 0.1], [0.20, 0.25, 0.25], "offset"),
]


@pytest.mark.basic
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("p0,p1,label", _PP_CASES, ids=[c[2] for c in _PP_CASES])
def test_pp_fd(ddf_session, p0, p1, label):
    """Point-point: gradient and Hessian of dist2 and barrier.

    Reference (FD methodology, threshold conventions, and PP grad/hess test):
        ipc-toolkit/tests/src/tests/distance/test_point_point.cpp
            TEST_CASE("Point-point distance gradient", ...)  -- uses
            fd::finite_gradient + fd::compare_gradient with default test_eps=1e-4.
        ipc-toolkit/tests/src/tests/distance/test_distance_type.cpp
            TEST_CASE("Point-point distance type", ...)
    """
    _, _, ddf = ddf_session
    R    = Geometry()
    sc_a = _pc([p0])
    sc_b = _pc([p1])
    gl   = [sc_a, sc_b]
    n    = 6

    def compute():
        ddf.compute_point_point_distance(R, sc_a, sc_b)

    compute()
    print(f"\n[PP/{label}]  dist2={_scalar(R, 'dist2'):.4f}  "
          f"barrier={_scalar(R, 'barrier')}")

    _check_dist2_barrier(compute, gl, R, n, test_hess=True, label=f"PP/{label}")


# ─────────────────────────────────────────────────────────────────────────────
# ═══════════════════════════════  POINT-EDGE  ════════════════════════════════
# ─────────────────────────────────────────────────────────────────────────────
# Fixed edge: E0=(−0.5, 0, 0), E1=(0.5, 0, 0)  — along the X axis
#
# Voronoi regions:
#   P_E   (interior): projection parameter t ∈ (0, 1)
#   P_E0  (vertex 0): t < 0
#   P_E1  (vertex 1): t > 1
# Boundaries:
#   t = 0  →  P_x = −0.5 = E0_x   (gradient C¹, Hessian skip)
#   t = 1  →  P_x =  0.5 = E1_x   (gradient C¹, Hessian skip)

_PE_EDGE = [[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]]

_PE_CASES = [
    # (p,                          label,             test_hess)
    ([0.0,   0.20, 0.0],           "P_E_centre",      True),
    ([0.2,   0.20, 0.10],          "P_E_off_axis",    True),
    ([-0.40, 0.20, 0.0],           "P_E_near_t0",     True),
    ([-0.50, 0.20, 0.0],           "boundary_t0",     False),  # C¹ boundary
    ([-0.65, 0.20, 0.0],           "P_E0_close",      True),
    ([ 0.40, 0.20, 0.0],           "P_E_near_t1",     True),
    ([ 0.50, 0.20, 0.0],           "boundary_t1",     False),  # C¹ boundary
    ([ 0.65, 0.20, 0.0],           "P_E1_close",      True),
]


@pytest.mark.basic
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("p,label,test_hess", _PE_CASES,
                         ids=[c[1] for c in _PE_CASES])
def test_pe_fd(ddf_session, p, label, test_hess):
    """Point-edge: gradient and Hessian across all Voronoi regions and boundaries.

    Reference (FD checks against analytical grad/hess, plus all 3 distance types
    P_E0, P_E1, P_E -- matches our P_E0_*, P_E1_*, P_E_* coverage):
        ipc-toolkit/tests/src/tests/distance/test_point_edge.cpp
            TEST_CASE("Point-edge distance gradient", ...)
              -- fd::finite_gradient + fd::compare_gradient(grad, fgrad)
            TEST_CASE("Point-edge distance hessian", ...)
              -- fd::finite_hessian + fd::compare_hessian(hess, fhess, 1e-2)
        ipc-toolkit/tests/src/tests/distance/test_distance_type.cpp
            TEST_CASE("Point-edge distance type", ...)
    Voronoi-boundary cases (test_hess=False at exactly t=0 / t=1) follow the
    same C¹-only convention noted by ipc-toolkit when the closest-feature type
    transitions.
    """
    _, _, ddf = ddf_session
    R       = Geometry()
    pt_sc   = _pc([p])
    edge_sc = _lm(_PE_EDGE)
    gl      = [pt_sc, edge_sc]
    n       = 9

    def compute():
        ddf.compute_point_edge_distance(R, pt_sc, edge_sc)

    compute()
    flag = _vec(R, "flag", 3)
    print(f"\n[PE/{label}]  dist2={_scalar(R, 'dist2'):.4f}  "
          f"flag={flag}  barrier={_scalar(R, 'barrier')}")

    _check_dist2_barrier(compute, gl, R, n, test_hess=test_hess,
                         label=f"PE/{label}")


# ─────────────────────────────────────────────────────────────────────────────
# ═══════════════════════════  POINT-TRIANGLE  ════════════════════════════════
# ─────────────────────────────────────────────────────────────────────────────
# Fixed triangle (in the y = 0 plane):
#   T0 = (−1, 0, −0.5),  T1 = (1, 0, −0.5),  T2 = (0, 0, 1)
# Centroid = (0, 0, 0).  Point P always has y = 0.2.
#
# Voronoi regions (7):
#   P_T                 — projection inside face
#   P_E0 (edge T0–T1)   — projection on edge T0T1 (bottom)
#   P_E1 (edge T1–T2)   — projection on edge T1T2 (right)
#   P_E2 (edge T2–T0)   — projection on edge T2T0 (left)
#   P_T0, P_T1, P_T2    — closest vertex regions
# Voronoi boundary examples: projection exactly on an edge or exactly at a vertex.

_PT_TRI = [[-1.0, 0.0, -0.5], [1.0, 0.0, -0.5], [0.0, 0.0, 1.0]]

_PT_CASES = [
    # (p,                              label,               test_hess)
    # ── P_T (face interior) ──────────────────────────────────────────────────
    ([0.0,   0.2,  0.0],               "P_T_centroid",      True),
    ([0.3,   0.2, -0.1],               "P_T_interior",      True),
    # ── near / on boundary P_T ↔ P_E0 (edge T0T1 at z = −0.5) ──────────────
    ([0.0,   0.2, -0.42],              "P_T_near_E0",       True),
    ([0.0,   0.2, -0.50],              "boundary_T_E0",     False),  # C¹ boundary
    ([0.0,   0.2, -0.58],              "P_E0_region",       True),
    # ── boundary P_T ↔ P_E1 (midpoint of T1–T2, projection=(0.5,0,0.25)) ────
    ([0.5,   0.2,  0.25],              "boundary_T_E1",     False),  # C¹ boundary
    # ── P_E1 region (edge T1–T2) ─────────────────────────────────────────────
    ([0.65,  0.2,  0.15],              "P_E1_region",       True),
    # ── boundary P_T ↔ P_E2 (midpoint of T2–T0, projection=(−0.5,0,0.25)) ───
    ([-0.5,  0.2,  0.25],              "boundary_T_E2",     False),  # C¹ boundary
    # ── P_E2 region (edge T2–T0) ─────────────────────────────────────────────
    ([-0.55, 0.2,  0.30],              "P_E2_region",       True),
    # ── T0 vertex (P_E0 / P_E2 / P_T0 junction) ─────────────────────────────
    ([-1.0,  0.2, -0.50],              "boundary_T_T0",     False),  # C¹ boundary
    ([-1.15, 0.2, -0.55],              "P_T0_region",       True),
    # ── T1 vertex (P_E0 / P_E1 / P_T1 junction) ─────────────────────────────
    ([1.0,   0.2, -0.50],              "boundary_T_T1",     False),  # C¹ boundary
    ([1.15,  0.2, -0.55],              "P_T1_region",       True),
    # ── T2 vertex (P_E1 / P_E2 / P_T2 junction) ─────────────────────────────
    ([0.0,   0.2,  1.00],              "boundary_T_T2",     False),  # C¹ boundary
    ([0.0,   0.2,  1.15],              "P_T2_region",       True),
]


@pytest.mark.basic
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize("p,label,test_hess", _PT_CASES,
                         ids=[c[1] for c in _PT_CASES])
def test_pt_fd(ddf_session, p, label, test_hess):
    """Point-triangle: gradient and Hessian across all 7 Voronoi regions and boundaries.

    Reference (FD checks of all PT distance types -- P_T0/T1/T2 (vertex),
    P_E0/E1/E2 (edge), P_T (face); our case set covers all 7 regions and the
    6 inter-region C¹ boundaries):
        ipc-toolkit/tests/src/tests/distance/test_point_triangle.cpp
            TEST_CASE("Point-triangle distance gradient", ...)
              -- fd::finite_gradient + fd::compare_gradient(grad, fgrad)
            TEST_CASE("Point-triangle distance hessian", ...)
              -- fd::finite_hessian + fd::compare_hessian(hess, fhess, 1e-2)
        ipc-toolkit/tests/src/tests/distance/test_distance_type.cpp
            TEST_CASE("Point-triangle distance type", ...)
    """
    _, _, ddf = ddf_session
    R      = Geometry()
    pt_sc  = _pc([p])
    tri_sc = _tm(_PT_TRI)
    gl     = [pt_sc, tri_sc]
    n      = 12

    def compute():
        ddf.compute_point_triangle_distance(R, pt_sc, tri_sc)

    compute()
    flag = _vec(R, "flag", 4)
    print(f"\n[PT/{label}]  dist2={_scalar(R, 'dist2'):.4f}  "
          f"flag={flag}  barrier={_scalar(R, 'barrier')}")

    _check_dist2_barrier(compute, gl, R, n, test_hess=test_hess,
                         label=f"PT/{label}")


# ─────────────────────────────────────────────────────────────────────────────
# ═══════════════════════════════  EDGE-EDGE  ═════════════════════════════════
# ─────────────────────────────────────────────────────────────────────────────
# Fixed edge A (current & rest): EA0 = (−0.5, 0, 0), EA1 = (0.5, 0, 0)
# Edge B varies; its rest mesh matches its initial current configuration.
#
# The mollifier e_k is active (e_k < 1) when the edges are near-parallel.
# All 9 EE Voronoi regions are tested:
#   EA_EB               — interior × interior (+ near-parallel with mollifier)
#   EA0_EB, EA1_EB      — vertex × interior
#   EA_EB0, EA_EB1      — interior × vertex
#   EA0_EB0, EA0_EB1,
#   EA1_EB0, EA1_EB1    — vertex × vertex
#
# Voronoi boundary cases (test_hess=False):
#   boundary_EAEB_EA0EB   — s_A = 0  (EA_EB ↔ EA0_EB)
#   boundary_EAEB_EA1EB   — s_A = 1  (EA_EB ↔ EA1_EB)
#   boundary_EAEB_EA_EB0  — t_B = 0  (EA_EB ↔ EA_EB0)
#   boundary_EAEB_EA_EB1  — t_B = 1  (EA_EB ↔ EA_EB1)
#   boundary_EA0_EB0      — s_A=0, t_B=0  (EA0_EB ↔ EA0_EB0)
#
# ``test_mollifier=True`` adds separate checks for e_k and its derivatives.

_EA = [[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]]

_EE_CASES = [
    # (eb0,                eb1,              label,                    test_hess, test_mollifier)
    # ── EA_EB: near-parallel (mollifier ON) ───────────────────────────────────
    ([-0.5, 0.2,  0.0],  [ 0.5, 0.2,  0.0], "parallel",               False, True),
    ([-0.5, 0.2,  0.01], [ 0.5, 0.2, -0.01],"near_parallel_tilt",     True,  True),
    # ── EA_EB: perpendicular (mollifier OFF, both edges interior) ─────────────
    ([ 0.0, 0.2, -0.5],  [ 0.0, 0.2,  0.5], "perp_EA_EB",             True,  False),
    # ── EA_EB ↔ EA0_EB boundary: s_A = 0 (EB at x = EA0.x = −0.5) ───────────
    #    unconstrained s_A = (EB.x − EA0.x)/|EA| = 0 ⟹ Hessian skip
    ([-0.5, 0.2, -0.3],  [-0.5, 0.2,  0.3], "boundary_EAEB_EA0EB",    False, False),
    # ── EA0_EB: EA0 vertex closest to EB interior (EB at x = −0.6) ───────────
    ([-0.6, 0.2, -0.3],  [-0.6, 0.2,  0.3], "EA0_EB",                 True,  False),
    # ── EA_EB ↔ EA1_EB boundary: s_A = 1 (EB at x = EA1.x = 0.5) ────────────
    ([ 0.5, 0.2, -0.3],  [ 0.5, 0.2,  0.3], "boundary_EAEB_EA1EB",    False, False),
    # ── EA1_EB: EA1 vertex closest to EB interior (EB at x = 0.6) ────────────
    ([ 0.6, 0.2, -0.3],  [ 0.6, 0.2,  0.3], "EA1_EB",                 True,  False),
    # ── EA_EB ↔ EA_EB0 boundary: t_B = 0 (EB0 projects onto EA interior) ─────
    #    project EA(s=0.5)=(0,0,0) onto EB=(0,0.2,0)→(0,0.2,0.3): t = 0 exactly
    ([ 0.0, 0.2,  0.0],  [ 0.0, 0.2,  0.3], "boundary_EAEB_EA_EB0",   False, False),
    # ── EA_EB0: EA interior closest to EB0 vertex ─────────────────────────────
    ([ 0.0, 0.2,  0.3],  [ 0.0, 0.2,  0.8], "EA_EB0",                 True,  False),
    # ── EA_EB ↔ EA_EB1 boundary: t_B = 1 (EB1 projects onto EA interior) ─────
    #    project EA(s=0.5)=(0,0,0) onto EB=(0,0.2,−0.3)→(0,0.2,0): t = 1 exactly
    ([ 0.0, 0.2, -0.3],  [ 0.0, 0.2,  0.0], "boundary_EAEB_EA_EB1",   False, False),
    # ── EA_EB1: EA interior closest to EB1 vertex ─────────────────────────────
    ([ 0.0, 0.2, -0.8],  [ 0.0, 0.2, -0.3], "EA_EB1",                 True,  False),
    # ── EA0_EB ↔ EA0_EB0 boundary ────────────────────────────────────────────
    #    t = (EA0 − EB0)·EB_dir = 0: EB0=(−0.5,0.2,0), EB_dir=(0.5,0,1)/‖‖
    ([-0.5, 0.2,  0.0],  [ 0.0, 0.2,  1.0], "boundary_EA0_EB0",       False, False),
    # ── Vertex-vertex regions ─────────────────────────────────────────────────
    # EA0_EB0: s_A < 0, t_B < 0  →  EB at x = −0.6, EB0 at z > 0
    ([-0.6, 0.2,  0.1],  [-0.6, 0.2,  1.0], "EA0_EB0",                True,  False),
    # EA0_EB1: s_A < 0, t_B > 1  →  EB at x = −0.6, EB1 at z < 0
    ([-0.6, 0.2, -1.0],  [-0.6, 0.2, -0.1], "EA0_EB1",                True,  False),
    # EA1_EB0: s_A > 1, t_B < 0  →  EB at x =  0.6, EB0 at z > 0
    ([ 0.6, 0.2,  0.1],  [ 0.6, 0.2,  1.0], "EA1_EB0",                True,  False),
    # EA1_EB1: s_A > 1, t_B > 1  →  EB at x =  0.6, EB1 at z < 0
    ([ 0.6, 0.2, -1.0],  [ 0.6, 0.2, -0.1], "EA1_EB1",                True,  False),
]


@pytest.mark.basic
@pytest.mark.skipif(skip_cuda_on_macos, reason=skip_cuda_on_macos_reason)
@pytest.mark.parametrize(
    "eb0,eb1,label,test_hess,test_mollifier", _EE_CASES,
    ids=[c[2] for c in _EE_CASES],
)
def test_ee_fd(ddf_session, eb0, eb1, label, test_hess, test_mollifier):
    """Edge-edge: gradient and Hessian across Voronoi regions, mollifier, and barrier.

    References (FD checks of EE distance + the C¹ EE-mollifier e_k):
        ipc-toolkit/tests/src/tests/distance/test_edge_edge.cpp
            TEST_CASE("Edge-edge distance",                       ...)
            TEST_CASE("Edge-edge distance !EA_EB",                ...)  -- vertex regions
            TEST_CASE("Edge-edge distance EA_EB",                 ...)  -- interior region
            TEST_CASE("Edge-edge distance parallel",              ...)  -- parallel limit
            TEST_CASE("Edge-edge distance gradient",              ...)
              -- fd::finite_gradient + fd::compare_gradient(grad, fgrad)
            TEST_CASE("Parallel edge-edge distance gradient",     ...)
              -- only "Almost parallel" SECTION runs; exact `Parallel` SECTION
                 is COMMENTED OUT and the Hessian CHECK is COMMENTED OUT.
                 We mirror this by setting test_hess=False for the `parallel`
                 case (dist2 Hessian is discontinuous at the parallel limit).
        ipc-toolkit/tests/src/tests/distance/test_edge_edge_mollifier.cpp
            TEST_CASE("Edge-Edge Cross Squarednorm", ...)   -- ‖(EA1-EA0)×(EB1-EB0)‖²
            TEST_CASE("Edge-Edge Mollifier Scalar",  ...)
              -- guards Hessian with `if (std::abs(x - eps_x) > 1e-6)` because
                 the smoothstep mollifier is only C¹ at the x = eps_x boundary;
                 we follow the same rule (e_k is C² at exact parallel x=0, so
                 e_k Hessian is checked there independently of test_hess).
            TEST_CASE("Edge-Edge Mollifier",         ...)   -- e_k FD grad/hess
        ipc-toolkit/tests/src/tests/distance/test_distance_type.cpp
            TEST_CASE("Edge-edge distance type", ...)       -- 9 EE Voronoi types
    """
    _, _, ddf = ddf_session
    R       = Geometry()
    ea_sc   = _lm(_EA)
    eb_sc   = _lm([eb0, eb1])
    ea_rest = _lm_rest(_EA)
    eb_rest = _lm_rest([eb0, eb1])
    gl      = [ea_sc, eb_sc]   # only current-position edges are perturbed
    n       = 12

    def compute():
        ddf.compute_edge_edge_distance(R, ea_sc, eb_sc, ea_rest, eb_rest)

    compute()
    ek_val = _scalar(R, "e_k")
    print(f"\n[EE/{label}]  dist2={_scalar(R,'dist2'):.4f}  "
          f"e_k={ek_val}  barrier={_scalar(R,'barrier')}")

    # ── dist2 + barrier ──────────────────────────────────────────────────────
    _check_dist2_barrier(compute, gl, R, n, test_hess=test_hess,
                         label=f"EE/{label}")

    # ── mollifier e_k (separate test when mollifier is active) ───────────────
    # e_k uses smoothstep (3t²−2t³, t = cross²/eps_x), which is:
    #   C∞ at t=0 (exact parallel)  → Hessian test always runs here
    #   C¹ only at t=1 (x = eps_x transition boundary)  → none of our cases land there
    # Therefore e_k Hessian is independent of test_hess (dist2 discontinuity).
    if test_mollifier:
        assert ek_val is not None, f"[EE/{label}] e_k attribute missing"
    if test_mollifier and ek_val < 1.0:

        err = schroeder_grad_test(
            compute, gl,
            lambda: _scalar(R, "e_k"),
            lambda: _vec(R, "e_k/grad", n),
            seed=4,
        )
        assert err is not None and err < _EK_THRESH, (
            f"[EE/{label}] e_k grad normalised-error={err:.3e} >= {_EK_THRESH}"
        )

        # e_k Hessian: always check — smoothstep is C² at t=0 (parallel)
        # and at t=1 (eps_x boundary), but none of our cases sit on t=1.
        err = schroeder_hess_test(
            compute, gl,
            lambda: _vec(R, "e_k/grad", n),
            lambda: _mat(R, "e_k/hess", n),
            seed=5,
        )
        assert err is not None and err < _EK_THRESH, (
            f"[EE/{label}] e_k hess normalised-error={err:.3e} >= {_EK_THRESH}"
        )
