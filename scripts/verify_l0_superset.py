"""
Verify the fundamental algebraic claim:
    MAS L0 cluster matrix == intra-cluster block of A (the full A_cluster).

If so, block-Jacobi with cluster blocks must be >= block-Jacobi with body blocks (ABDDiag).
"""
import numpy as np
import json
import sys
from pathlib import Path

DBG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case\95_abd_mas_cloth_benchmark.cpp\mas\debug\cuda\affine_body\abd_mas_preconditioner.cu")
print(f"Loading from: {DBG}")

meta = json.load(open(DBG / "mas_cluster_meta.f1.n0.json"))
N = meta["total_nodes"]
PAD = meta["total_clusters"]
BS = meta["banksize"]
LEV = meta["level_num"]
levels = meta["levels"]
real_to_part = np.array(meta["real_to_part"])
part_to_real = np.array(meta["part_to_real"])
NUM_BLOCKS = PAD // BS
DIM = BS * 3

print(f"N(fine)={N} banks={NUM_BLOCKS} BS={BS}")

# ---- Load fine H ----
trips = []
with open(DBG / "abd_bcoo.f1.n0.txt") as f:
    for line in f:
        if line.startswith("#"): continue
        s = line.strip().split()
        if len(s) < 11: continue
        r, c = int(s[0]), int(s[1])
        v = np.array([float(x) for x in s[2:11]]).reshape(3, 3)
        trips.append((r, c, v))

H = np.zeros((3*N, 3*N))
for r, c, v in trips:
    H[3*r:3*r+3, 3*c:3*c+3] += v
    if r != c:
        H[3*c:3*c+3, 3*r:3*r+3] += v.T

print(f"H: |H|_F = {np.linalg.norm(H):.4e}, sym = {np.linalg.norm(H - H.T):.4e}")

# ---- Load L0 cluster matrices ----
def load_mtx(path):
    arr = [np.zeros((DIM, DIM)) for _ in range(NUM_BLOCKS)]
    with open(path) as f:
        for line in f:
            if line.startswith("%"): continue
            s = line.strip().split()
            if len(s) < 3: continue
            try:
                i, j, v = int(s[0])-1, int(s[1])-1, float(s[2])
            except ValueError:
                continue
            cid = i // DIM
            if cid != j // DIM or cid >= NUM_BLOCKS: continue
            arr[cid][i % DIM, j % DIM] = v
    for c in range(NUM_BLOCKS):
        M = arr[c]
        Ms = np.zeros_like(M)
        for br in range(BS):
            for bc in range(br, BS):
                blk = M[br*3:br*3+3, bc*3:bc*3+3]
                Ms[br*3:br*3+3, bc*3:bc*3+3] = blk
                if bc > br:
                    Ms[bc*3:bc*3+3, br*3:br*3+3] = blk.T
        arr[c] = Ms
    return arr

hess = load_mtx(DBG / "mas_cluster_hess.f1.n0.mtx")

# ---- For L0 clusters, check: cluster_hess == intra-cluster block of H ----
# L0 level occupies banks 0..(L1_offset/BS - 1)
L1 = next(L for L in levels if L["level"] == 1)
L0_OFF = 0
L0_END = L1["offset"]
L0_BANKS = L0_END // BS

print(f"L0 banks: {L0_BANKS}")

# For each L0 cluster, the 16 nodes are at padded positions [bank*BS, (bank+1)*BS).
# Map these to real fine indices via part_to_real.
max_err = 0.0
worst_bank = -1
for bank in range(L0_BANKS):
    pad_positions = list(range(bank*BS, (bank+1)*BS))
    real_of = [part_to_real[p] for p in pad_positions]

    # Build expected intra-cluster block of H (48x48): using only real positions, padded = zero
    expected = np.zeros((DIM, DIM))
    for lr, rr in enumerate(real_of):
        if rr < 0: continue
        for lc, rc in enumerate(real_of):
            if rc < 0: continue
            expected[3*lr:3*lr+3, 3*lc:3*lc+3] = H[3*rr:3*rr+3, 3*rc:3*rc+3]

    actual = hess[bank]
    err = np.max(np.abs(expected - actual))
    if err > max_err:
        max_err = err
        worst_bank = bank
        worst_diff = expected - actual
        worst_exp = expected
        worst_act = actual

print(f"\n|cluster_hess - intra_cluster_H|_max across all {L0_BANKS} L0 banks = {max_err:.4e}")
print(f"Worst bank: {worst_bank}")
if max_err > 1e-6:
    # show which blocks differ
    diff = worst_diff
    for br in range(BS):
        for bc in range(BS):
            d = np.abs(diff[3*br:3*br+3, 3*bc:3*bc+3]).max()
            if d > 1e-6:
                pad_r = worst_bank*BS + br
                pad_c = worst_bank*BS + bc
                rr = part_to_real[pad_r]
                rc = part_to_real[pad_c]
                body_r = rr // 4 if rr >= 0 else -1
                body_c = rc // 4 if rc >= 0 else -1
                e_norm = np.linalg.norm(worst_exp[3*br:3*br+3, 3*bc:3*bc+3])
                a_norm = np.linalg.norm(worst_act[3*br:3*br+3, 3*bc:3*bc+3])
                print(f"  ({br},{bc}): diff={d:.3e}  expected={e_norm:.3e}  actual={a_norm:.3e}  real=({rr},{rc}) bodies=({body_r},{body_c})")
else:
    print("  MAS L0 cluster matrices ARE exactly the intra-cluster blocks of A. OK.")

# ---- Now build the two preconditioners and compare their spectra on a SMALL system ----
# For cloth 900 bodies = 10800 DOFs, full eigvals is slow. Do it sparse.
# Build M_Diag^{-1}: 12x12 per-body inverse (excluding fixed bodies - set to identity padded)
print("\n=== Building preconditioners ===")
print("M_Diag (per-body 12x12 block-Jacobi)...")
# Per-body 12x12 self = H at positions [body*4, (body+1)*4) x [body*4, (body+1)*4)
N_BODIES = N // 4
M_diag_inv = np.zeros((3*N, 3*N))
for b in range(N_BODIES):
    blk = H[3*b*4:3*(b+1)*4, 3*b*4:3*(b+1)*4]
    # If diag is zero (fixed body), use identity to avoid singular inverse
    if np.any(np.diag(blk) == 0):
        continue  # fixed body -> inverse contribution is zero
    M_diag_inv[3*b*4:3*(b+1)*4, 3*b*4:3*(b+1)*4] = np.linalg.inv(blk)

print("M_MAS L0 (per-cluster 48x48 block-Jacobi)...")
M_mas_inv = np.zeros((3*N, 3*N))
for bank in range(L0_BANKS):
    pad_positions = list(range(bank*BS, (bank+1)*BS))
    real_of = [part_to_real[p] for p in pad_positions]
    valid_real = [r for r in real_of if r >= 0]
    if not valid_real: continue
    # Extract the intra-cluster block over real positions only
    n_real = len(valid_real)
    A_sub = np.zeros((3*n_real, 3*n_real))
    for i, ri in enumerate(valid_real):
        for j, rj in enumerate(valid_real):
            A_sub[3*i:3*i+3, 3*j:3*j+3] = H[3*ri:3*ri+3, 3*rj:3*rj+3]
    # Skip fixed-heavy clusters
    if np.any(np.diag(A_sub) == 0):
        # replace zero diags with 1 to avoid singular inverse
        for k in range(A_sub.shape[0]):
            if A_sub[k, k] == 0:
                A_sub[k, k] = 1.0
    try:
        A_sub_inv = np.linalg.inv(A_sub)
    except np.linalg.LinAlgError:
        continue
    for i, ri in enumerate(valid_real):
        for j, rj in enumerate(valid_real):
            M_mas_inv[3*ri:3*ri+3, 3*rj:3*rj+3] = A_sub_inv[3*i:3*i+3, 3*j:3*j+3]

# ---- Build the same M_MAS^{-1} but using the DUMPED GPU cluster_inv ----
print("M_MAS L0 (from GPU dumped cluster_inv)...")
inv_dump = load_mtx(DBG / "mas_cluster_inv.f1.n0.mtx")
M_mas_inv_gpu = np.zeros((3*N, 3*N))
for bank in range(L0_BANKS):
    pad_positions = list(range(bank*BS, (bank+1)*BS))
    real_of = [part_to_real[p] for p in pad_positions]
    # Even for padded positions, the GPU inverts the full 48x48 (identity-padded)
    # The inverse at real positions is what we use.
    for i, ri in enumerate(real_of):
        if ri < 0: continue
        for j, rj in enumerate(real_of):
            if rj < 0: continue
            M_mas_inv_gpu[3*ri:3*ri+3, 3*rj:3*rj+3] = inv_dump[bank][3*i:3*i+3, 3*j:3*j+3]

diff_Mmas = np.max(np.abs(M_mas_inv - M_mas_inv_gpu))
print(f"|M_MAS_numpy - M_MAS_gpu|_max = {diff_Mmas:.4e}")

# ---- Compare spectra ----
print("\n=== Spectrum comparison (subset of eigenvalues via iterative solver) ===")
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import eigsh

# Skip fixed body DOFs
free_dofs = []
for b in range(N_BODIES):
    blk = H[3*b*4:3*(b+1)*4, 3*b*4:3*(b+1)*4]
    if not np.any(np.diag(blk) == 0):
        for k in range(12):
            free_dofs.append(3*b*4 + k)
free_dofs = np.array(free_dofs)
print(f"Free DOFs: {len(free_dofs)} / {3*N}")

H_free = H[free_dofs][:, free_dofs]
Md_free = M_diag_inv[free_dofs][:, free_dofs]
Mm_free = M_mas_inv[free_dofs][:, free_dofs]
Mm_gpu_free = M_mas_inv_gpu[free_dofs][:, free_dofs]

# Compute extreme eigvals of M^{-1} A
def eigs_extreme(Minv, A, name):
    if A.shape[0] > 3000:
        print(f"  {name}: skipping dense eigvals (size {A.shape[0]} too large)")
        return
    MA = Minv @ A
    eigs = np.linalg.eigvals(MA).real
    pos = eigs[eigs > 1e-10]
    if len(pos) == 0:
        print(f"  {name}: no positive eigs")
        return
    print(f"  {name}: min={pos.min():.4e}, max={pos.max():.4e}, cond={pos.max()/pos.min():.4e}, n_pos={len(pos)}, n_neg={(eigs < -1e-10).sum()}")

eigs_extreme(Md_free, H_free, "M_Diag (12x12 per-body)")
eigs_extreme(Mm_free, H_free, "M_MAS  (48x48 per-cluster)")

# ---- PCG simulation to count iterations until convergence ----
def pcg_iter_count(A, Minv, tol_rate=1e-3, maxit=1000):
    """Matches runtime PCG: stop when rr / rr0 <= tol_rate (not squared)."""
    n = A.shape[0]
    np.random.seed(0)
    b = np.random.randn(n)
    x = np.zeros(n)
    r = b - A @ x
    z = Minv @ r
    p = z.copy()
    rz = r @ z
    rr0 = r @ r
    for k in range(maxit):
        Ap = A @ p
        alpha = rz / (p @ Ap)
        x = x + alpha * p
        r = r - alpha * Ap
        rr = r @ r
        if rr / rr0 <= tol_rate:
            return k + 1
        z = Minv @ r
        rz_new = r @ z
        beta = rz_new / rz
        rz = rz_new
        p = z + beta * p
    return maxit

for tol_rate in [1e-12]:
    print(f"\n=== PCG iteration count to rr/rr0 <= {tol_rate} ===")
    print(f"  M_Diag:           {pcg_iter_count(H_free, Md_free, tol_rate, maxit=4000)}")
    print(f"  M_MAS L0 (numpy): {pcg_iter_count(H_free, Mm_free, tol_rate, maxit=4000)}")
    print(f"  M_MAS L0 (GPU):   {pcg_iter_count(H_free, Mm_gpu_free, tol_rate, maxit=4000)}")
