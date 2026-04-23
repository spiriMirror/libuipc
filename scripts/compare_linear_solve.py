"""
compare_linear_solve.py - Compare Diag vs MAS PCG linear solve dumps.

Usage:
    python compare_linear_solve.py [<diag_output_dir> <mas_output_dir> [frame [newton]]]

What it does:
  1. Load the BCOO global Hessian A and gradient b (from dump_linear_system)
  2. Solve A*x = b with scipy sparse solver (ground truth)
  3. Load PCG solution x_diag and x_mas (from dump_linear_system after PCG)
  4. Check if both PCG solutions match the scipy solution (convergence correctness)
  5. Load z0 (first preconditioner response) for both Diag and MAS
  6. Compare alignment of z0 with b and with scipy solution
  7. Compute and compare ||Ax - b|| residuals (measure of convergence quality)
"""

import sys, pathlib
import numpy as np
import scipy.io as sio
import scipy.sparse as sp
import scipy.sparse.linalg as spla

# ---- paths ------------------------------------------------------------------
BASE = pathlib.Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case"
    r"\94_abd_mas_linear_solve_debug.cpp"
)
PCG_SUBDIR = pathlib.Path("debug") / "cuda" / "linear_system" / "linear_pcg.cu"
SYS_SUBDIR = pathlib.Path("debug") / "cuda" / "linear_system" / "global_linear_system.cu"

def load_vec(path):
    path = pathlib.Path(path)
    if not path.exists():
        return None
    m = sio.mmread(str(path))
    if hasattr(m, 'toarray'):
        return m.toarray().flatten().astype(np.float64)
    return np.asarray(m, dtype=np.float64).flatten()

def load_mat(path):
    """Load a sparse Matrix Market file as scipy CSR."""
    path = pathlib.Path(path)
    if not path.exists():
        return None
    m = sio.mmread(str(path))
    # BCoo stores upper-triangle; symmetrize to get full SPD matrix
    coo = m.tocsr() if hasattr(m, 'tocsr') else sp.csr_matrix(m)
    csr = sp.csr_matrix(coo)
    # Symmetrize: A = csr + csr.T - diag(csr)
    diag = sp.diags(csr.diagonal())
    A_sym = csr + csr.T - diag
    return A_sym.astype(np.float64)

def rel_err(a, b):
    denom = max(np.linalg.norm(b), 1e-300)
    return np.linalg.norm(a - b) / denom

def cosine(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-300 or nb < 1e-300:
        return float('nan')
    return float(np.dot(a, b) / (na * nb))

def analyze_run(label, root, frame=1, newton=0):
    root   = pathlib.Path(root)
    pcg    = root / PCG_SUBDIR
    sysdir = root / SYS_SUBDIR

    print(f"\n{'='*60}")
    print(f"  {label}: {root}")
    print(f"  Frame={frame}  Newton={newton}")
    print(f"{'='*60}")

    # --- load vectors --------------------------------------------------------
    A = load_mat (sysdir / f"A.{frame}.{newton}.mtx")
    b = load_vec (sysdir / f"b.{frame}.{newton}.mtx")
    x = load_vec (sysdir / f"x.{frame}.{newton}.mtx")

    r0 = load_vec(pcg / f"r.{frame}.{newton}.0.mtx")
    z0 = load_vec(pcg / f"z.{frame}.{newton}.0.mtx")
    p1 = load_vec(pcg / f"p.{frame}.{newton}.1.mtx")
    Ap1= load_vec(pcg / f"Ap.{frame}.{newton}.1.mtx")

    if b is None:
        print("  b.mtx NOT FOUND - were dumps enabled?")
        return None
    print(f"  DOFs = {len(b)}")

    # --- scipy ground truth --------------------------------------------------
    x_true = None
    if A is not None:
        print(f"  Hessian: {A.shape}  nnz={A.nnz}")
        # Check symmetry
        diff = A - A.T
        sym_err = abs(diff).max()
        print(f"  Symmetry ||A - A^T||_inf = {sym_err:.2e}")
        try:
            x_true = spla.spsolve(A, b)
            res = np.linalg.norm(A @ x_true - b) / max(np.linalg.norm(b), 1e-300)
            print(f"  scipy.spsolve residual     = {res:.2e}")
        except Exception as e:
            print(f"  scipy solve failed: {e}")

    # --- PCG solution quality ------------------------------------------------
    if x is not None and A is not None:
        res_x = np.linalg.norm(A @ x - b) / max(np.linalg.norm(b), 1e-300)
        print(f"  PCG solution residual    = {res_x:.2e}  (||Ax-b||/||b||)")
        if x_true is not None:
            print(f"  rel_err(x, x_true)       = {rel_err(x, x_true):.2e}")
    else:
        print("  x.mtx or A.mtx not found")

    # --- preconditioner quality (z0) -----------------------------------------
    if z0 is not None:
        cos_z0_b = cosine(z0, b)
        rz0 = float(np.dot(b, z0))  # r0 = b since x starts at 0
        print(f"\n  First precond response (z0 = M^{{-1}} b):")
        print(f"    ||z0|| = {np.linalg.norm(z0):.4e}")
        print(f"    rz0 = b^T z0 = {rz0:.6e}  (PCG stopping threshold = tol * |rz0|)")
        print(f"    cos(z0, b) = {cos_z0_b:.6f}  "
              f"{'[GOOD: >0]' if cos_z0_b > 0 else '[BAD: negative -> broken precond!]'}")
        if x_true is not None:
            print(f"    cos(z0, x_true) = {cosine(z0, x_true):.6f}  (vs true solution)")
        if A is not None:
            # Check if z0 is a valid preconditioner response: z0 should ≈ A^{-1}*b
            # Measure alignment with true solution direction
            if x_true is not None:
                # How much of the true solution is captured by z0?
                norm_true = np.linalg.norm(x_true)
                proj = np.dot(z0, x_true) / max(norm_true**2, 1e-300)
                print(f"    z0 projection on x_true = {proj:.4f}  (1.0 = perfect)")

    # --- Ap consistency check ------------------------------------------------
    if p1 is not None and Ap1 is not None and A is not None:
        Ap1_computed = A @ p1
        err = rel_err(Ap1_computed, Ap1)
        print(f"\n  SpMV check (A*p1 vs dumped Ap1):")
        print(f"    rel_err(A*p1, dumped_Ap1) = {err:.2e}  "
              f"{'[MATCH]' if err < 1e-4 else '[MISMATCH: Hessian assembly error!]'}")

    return {'A': A, 'b': b, 'x': x, 'x_true': x_true, 'r0': r0, 'z0': z0, 'p1': p1, 'Ap1': Ap1}


def compare(d_diag, d_mas):
    if d_diag is None or d_mas is None:
        print("\nCannot compare: missing data from one run.")
        return

    print(f"\n{'='*60}")
    print("  COMPARISON: Diag vs MAS")
    print(f"{'='*60}")

    b_diag = d_diag['b']
    b_mas  = d_mas['b']
    err_b  = rel_err(b_diag, b_mas)
    print(f"  rel_err(b_diag, b_mas) = {err_b:.2e}  "
          f"{'[MATCH: same Hessian + same DOF state]' if err_b < 1e-10 else '[MISMATCH: different gradients => different physical state!]'}")

    A_diag = d_diag['A']
    A_mas  = d_mas['A']
    if A_diag is not None and A_mas is not None:
        diff_A = A_diag - A_mas
        err_A  = abs(diff_A).max()
        nnz_diag = A_diag.nnz
        nnz_mas  = A_mas.nnz
        print(f"  ||A_diag - A_mas||_inf = {err_A:.2e}  nnz_diag={nnz_diag} nnz_mas={nnz_mas}")
        if err_A > 1e-8:
            print("  [!] HESSIANS DIFFER - possible BCOO pollution in MAS path!")

    x_diag  = d_diag['x']
    x_mas   = d_mas['x']
    xt_diag = d_diag['x_true']
    xt_mas  = d_mas['x_true']

    if x_diag is not None and x_mas is not None:
        err_x = rel_err(x_diag, x_mas)
        print(f"\n  PCG final solutions (fully converged at tol=1e-12):")
        print(f"    rel_err(x_diag, x_mas) = {err_x:.2e}  "
              f"{'[MATCH: both converged to same answer]' if err_x < 1e-6 else '[DIFFER: one did not converge!]'}")

    z0_diag = d_diag['z0']
    z0_mas  = d_mas['z0']
    if z0_diag is not None and z0_mas is not None:
        rz0_diag = float(np.dot(b_diag, z0_diag))
        rz0_mas  = float(np.dot(b_diag, z0_mas))
        print(f"\n  Preconditioner first response (z0 = M^{{-1}} b):")
        print(f"    rz0_diag = b^T z0_diag = {rz0_diag:.6e}")
        print(f"    rz0_mas  = b^T z0_mas  = {rz0_mas:.6e}")
        print(f"    rz0_mas / rz0_diag     = {rz0_mas / max(abs(rz0_diag), 1e-300):.4f}x")
        print(f"    (stopping tol=1e-3 is {rz0_mas/max(abs(rz0_diag),1e-300):.1f}x looser for MAS in absolute terms)")
        print(f"    cos(z0_diag, b) = {cosine(z0_diag, b_diag):.6f}")
        print(f"    cos(z0_mas,  b) = {cosine(z0_mas,  b_diag):.6f}")
        ratio = np.linalg.norm(z0_mas) / max(np.linalg.norm(z0_diag), 1e-300)
        print(f"    ||z0_mas|| / ||z0_diag|| = {ratio:.4f}")
        if xt_diag is not None:
            print(f"    cos(z0_diag, x_true) = {cosine(z0_diag, xt_diag):.6f}")
            print(f"    cos(z0_mas,  x_true) = {cosine(z0_mas,  xt_diag):.6f}")


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        diag_dir = sys.argv[1]
        mas_dir  = sys.argv[2]
    else:
        diag_dir = str(BASE / "diag")
        mas_dir  = str(BASE / "mas")

    frame  = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    newton = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    d_diag = analyze_run("DIAG", diag_dir, frame, newton)
    d_mas  = analyze_run("MAS",  mas_dir,  frame, newton)
    compare(d_diag, d_mas)
