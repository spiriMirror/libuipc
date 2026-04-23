"""Show the eigenvalue distribution of M_Diag^{-1} A vs M_MAS^{-1} A.

If MAS has a more spread-out / less-clustered spectrum, that explains why
its PCG iteration count can be higher even when its cond number is lower.
"""
import numpy as np
import json
import sys
from pathlib import Path

DBG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case\95_abd_mas_cloth_benchmark.cpp\lockstep\mas_L1\debug\cuda\affine_body\abd_mas_preconditioner.cu")
print(f"Loading: {DBG}")

meta = json.load(open(DBG / "mas_cluster_meta.f1.n0.json"))
N = meta["total_nodes"]
PAD = meta["total_clusters"]
BS = meta["banksize"]
levels = meta["levels"]
real_to_part = np.array(meta["real_to_part"])
part_to_real = np.array(meta["part_to_real"])
NUM_BLOCKS = PAD // BS
DIM = BS * 3

print(f"N(fine)={N}  total DOFs={3*N}")
if 3 * N > 2500:
    print("System too large for dense eigvals; aborting (use smaller grid)")
    sys.exit(0)

# Load fine H
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

# Build M_Diag (per-body 12x12)
N_BODIES = N // 4
M_diag_inv = np.zeros((3*N, 3*N))
for b in range(N_BODIES):
    blk = H[3*b*4:3*(b+1)*4, 3*b*4:3*(b+1)*4]
    if not np.any(np.diag(blk) == 0):
        M_diag_inv[3*b*4:3*(b+1)*4, 3*b*4:3*(b+1)*4] = np.linalg.inv(blk)

# Build M_MAS L0 (per-cluster 48x48)
L1_off = next(L for L in levels if L["level"] == 1)["offset"]
L0_BANKS = L1_off // BS
M_mas_inv = np.zeros((3*N, 3*N))
for bank in range(L0_BANKS):
    valid_real = [int(part_to_real[bank*BS + p]) for p in range(BS)
                  if part_to_real[bank*BS + p] >= 0]
    if not valid_real: continue
    n_real = len(valid_real)
    A_sub = np.zeros((3*n_real, 3*n_real))
    for i, ri in enumerate(valid_real):
        for j, rj in enumerate(valid_real):
            A_sub[3*i:3*i+3, 3*j:3*j+3] = H[3*ri:3*ri+3, 3*rj:3*rj+3]
    A_sub_inv = np.linalg.inv(A_sub)
    for i, ri in enumerate(valid_real):
        for j, rj in enumerate(valid_real):
            M_mas_inv[3*ri:3*ri+3, 3*rj:3*rj+3] = A_sub_inv[3*i:3*i+3, 3*j:3*j+3]

# Compute spectra
print("\nComputing eigenvalues of M^{-1} A ...")
eigs_diag = np.linalg.eigvals(M_diag_inv @ H).real
eigs_mas  = np.linalg.eigvals(M_mas_inv  @ H).real
eigs_diag = np.sort(eigs_diag[eigs_diag > 1e-12])
eigs_mas  = np.sort(eigs_mas[eigs_mas  > 1e-12])

print(f"\nM_Diag^-1 A:  N={len(eigs_diag)}  cond={eigs_diag[-1]/eigs_diag[0]:.4e}")
print(f"M_MAS^-1  A:  N={len(eigs_mas)}   cond={eigs_mas[-1]/eigs_mas[0]:.4e}")

print(f"\nDiag eigvals: min={eigs_diag[0]:.3e}  max={eigs_diag[-1]:.3e}")
print(f"MAS  eigvals: min={eigs_mas[0]:.3e}   max={eigs_mas[-1]:.3e}")

# Histogram of log10 eigenvalues -- where are the eigenvalues clustered?
def hist(eigs, name, n_bins=20):
    log10e = np.log10(eigs)
    lo, hi = log10e.min(), log10e.max()
    bins = np.linspace(lo, hi, n_bins+1)
    counts, _ = np.histogram(log10e, bins)
    print(f"\n{name} eigenvalue distribution (log10 scale):")
    for i in range(n_bins):
        bar = "#" * int(50 * counts[i] / max(counts.max(), 1))
        print(f"  {bins[i]:7.3f} .. {bins[i+1]:7.3f}: {counts[i]:5d}  {bar}")

hist(eigs_diag, "M_Diag^-1 A")
hist(eigs_mas,  "M_MAS^-1 A")

# Number of "effectively distinct" clusters: count eigvals separated by > some gap
def num_clusters(eigs, gap_factor=1.5):
    """Count clusters where adjacent eigenvalues differ by < gap_factor."""
    sorted_eigs = np.sort(eigs)
    n = 1
    for i in range(1, len(sorted_eigs)):
        if sorted_eigs[i] / sorted_eigs[i-1] > gap_factor:
            n += 1
    return n

for gap in [1.1, 1.5, 2.0, 5.0]:
    nd = num_clusters(eigs_diag, gap)
    nm = num_clusters(eigs_mas, gap)
    print(f"\nDistinct eigenvalue clusters (gap > {gap}x):")
    print(f"  M_Diag^-1 A:  {nd}")
    print(f"  M_MAS^-1  A:  {nm}")

# PCG iteration count (matches runtime convergence behavior)
def pcg_iter(A, Minv, tol=1e-12, maxit=2000):
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
        if rr / rr0 <= tol:
            return k + 1
        z = Minv @ r
        rz_new = r @ z
        beta = rz_new / rz
        rz = rz_new
        p = z + beta * p
    return maxit

for tol in [1e-3, 1e-6, 1e-9, 1e-12]:
    print(f"\nPCG iters to rr/rr0 <= {tol}:")
    print(f"  M_Diag: {pcg_iter(H, M_diag_inv, tol)}")
    print(f"  M_MAS:  {pcg_iter(H, M_mas_inv,  tol)}")
