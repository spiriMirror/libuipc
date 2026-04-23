"""Compute spectrum of M^{-1} A for fine-only and multi-level MAS to find the algebraic issue."""
import numpy as np
import json
from pathlib import Path
from scipy.sparse import csr_matrix, lil_matrix

import sys
DBG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case\93_abd_mas_vs_diag_benchmark.cpp\mas\debug\cuda\affine_body\abd_mas_preconditioner.cu")
print(f"Loading from: {DBG}")
meta = json.load(open(DBG / "mas_cluster_meta.f1.n0.json"))
N = meta["total_nodes"]
PAD = meta["total_clusters"]
BS  = meta["banksize"]
LEV = meta["level_num"]
levels = meta["levels"]
real_to_part = np.array(meta["real_to_part"])
going_next   = np.array(meta["going_next"])

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

print(f"Fine H: {H.shape}, |H|_F = {np.linalg.norm(H):.4e}, sym err = {np.linalg.norm(H - H.T):.4e}")

# Diagonal magnitudes per sub-node
print("\n=== Sub-node diagonal magnitudes (per body's 4 sub-nodes) ===")
for body in [0, 1, 50, 99]:
    print(f"  body {body}:")
    for s in range(4):
        node = body * 4 + s
        if 3*node + 3 > 3*N: continue
        diag_block = H[3*node:3*node+3, 3*node:3*node+3]
        print(f"    sub-node {s} (DOF type {'t' if s==0 else f'A{s}'}): diag norm = {np.linalg.norm(diag_block):.4e}")

# Build R_0 (fine block Jacobi by 16-node clusters) and R_1, R_2, R_3 restrictions
NUM_BLOCKS = PAD // BS
DIM = BS * 3

def load_cluster_mtx(path):
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

cluster_inv = load_cluster_mtx(DBG / "mas_cluster_inv.f1.n0.mtx")

# Build M_inv per level by applying R^T A_inv R with R = sum-restriction.
# For L0: R_0 maps fine real index -> padded fine pos.
# Padded fine pos of node a is `real_to_part[a]`.
# Within a cluster bank b, padded pos b*BS + lr.
# So R_0[3*pad + k, 3*a + k] = 1 if a maps to pad.

# fine "block jacobi" preconditioner: R_0 is identity-like (not aggregating).
# Each fine real `a` has a UNIQUE padded position. Cluster_inv restricted to real positions = fine block Jacobi.
def build_M_inv_L(level_idx):
    """Build M_L^{-1} = R_L^T A_L^{-1} R_L as 3N x 3N dense."""
    if level_idx == 0:
        coarse_of = real_to_part.copy()
    else:
        coarse_of = np.full(N, -1, dtype=int)
        for a in range(N):
            cur = a
            ok = True
            for l in range(level_idx):
                cur = going_next[cur]
                if cur < 0:
                    ok = False
                    break
            if ok:
                coarse_of[a] = cur

    # group fine nodes by their cluster bank at this level
    bank_to_nodes = {}
    for a in range(N):
        pa = coarse_of[a]
        if pa < 0: continue
        ba = pa // BS
        bank_to_nodes.setdefault(ba, []).append(a)

    Minv = np.zeros((3 * N, 3 * N))
    for ba, nodes in bank_to_nodes.items():
        for a in nodes:
            la = coarse_of[a] % BS
            for b in nodes:
                lb = coarse_of[b] % BS
                Minv[3*a:3*a+3, 3*b:3*b+3] += cluster_inv[ba][3*la:3*la+3, 3*lb:3*lb+3]
    return Minv

# Full multi-level M^{-1} = sum over levels
M_invs = {}
for lvl in range(LEV):  # levels 0 to LEV-1
    print(f"Building M_inv for level {lvl}...")
    M_invs[lvl] = build_M_inv_L(lvl)
    print(f"  |M_{lvl}^{{-1}}|_F = {np.linalg.norm(M_invs[lvl]):.4e}")

# Compute spectrum of M^{-1} A for L0 only, L0+L1, L0+L1+L2, full
print("\n=== Spectrum of M^{-1} A (eigenvalues) ===")
H_dense = H

def eigvals_summary(Minv, name):
    MA = Minv @ H_dense
    eigs = np.linalg.eigvals(MA)
    eigs = np.real(eigs)  # symmetric => real
    pos = eigs[eigs > 1e-10]
    if len(pos) == 0:
        print(f"  {name}: no positive eigenvalues!")
        return
    print(f"  {name}: min={pos.min():.4e}, max={pos.max():.4e}, cond={pos.max()/pos.min():.4e}, n_pos={len(pos)}, n_neg={(eigs < -1e-10).sum()}")

eigvals_summary(M_invs[0], "L0 only (block Jacobi)")
M_inv_total = M_invs[0].copy()
for lvl in range(1, LEV):
    M_inv_total += M_invs[lvl]
    eigvals_summary(M_inv_total, f"L0+...+L{lvl}")

# ---- Try damped multi-level: w_L = decreasing factor for coarse ----
print("\n=== Damped multi-level spectra (w_L applied to coarse contributions) ===")
for w in [0.5, 0.3, 0.2, 0.1, 0.05]:
    M_damped = M_invs[0].copy()
    for lvl in range(1, LEV):
        M_damped += w * M_invs[lvl]
    eigvals_summary(M_damped, f"L0 + {w}*(L1+L2+L3)")

# Per-level damping (w_L = 1/L)
print("\n=== Per-level weights w_L = 1/(L+1) ===")
M_damped = M_invs[0].copy()
for lvl in range(1, LEV):
    w = 1.0 / (lvl + 1)
    M_damped += w * M_invs[lvl]
eigvals_summary(M_damped, "1/(L+1)")

# Per-level damping (w_L = 0.5^L)
print("\n=== Geometric damping w_L = 0.5^L ===")
M_damped = M_invs[0].copy()
for lvl in range(1, LEV):
    w = 0.5 ** lvl
    M_damped += w * M_invs[lvl]
eigvals_summary(M_damped, "0.5^L")

# Compare to ABDDiag (per-body 12x12 inverse)
print("\n=== Spectrum of ABDDiag^{-1} A ===")
M_inv_abddiag = np.zeros((3*N, 3*N))
for body in range(N // 4):
    block = H_dense[3*body*4:3*(body+1)*4, 3*body*4:3*(body+1)*4]
    M_inv_abddiag[3*body*4:3*(body+1)*4, 3*body*4:3*(body+1)*4] = np.linalg.inv(block)
eigvals_summary(M_inv_abddiag, "ABDDiag (per-body 12x12 block inv)")
