"""
Verify MAS coarse Hessian construction is correct Galerkin (R H R^T).

Loads:
  - abd_bcoo.f1.n0.txt        : fine BCOO triplets
  - mas_cluster_meta.f1.n0.json : going_next, real_to_part, level_sizes, part_to_real
  - mas_cluster_hess.f1.n0.mtx  : per-cluster 48x48 dense Hessians (block-upper-tri stored)

Builds the full fine H_0 (sym), the L1 restriction R_1, and checks H_1_actual = R_1 H_0 R_1^T.
"""
import json, sys, os
import numpy as np
from scipy.sparse import csr_matrix, lil_matrix
from pathlib import Path

DBG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case"
    r"\93_abd_mas_vs_diag_benchmark.cpp\mas\debug\cuda\affine_body"
    r"\abd_mas_preconditioner.cu")

meta = json.load(open(DBG / "mas_cluster_meta.f1.n0.json"))
N = meta["total_nodes"]              # fine real nodes
PAD = meta["total_clusters"]         # padded total positions across all levels
BS  = meta["banksize"]
LEV = meta["level_num"]
levels = meta["levels"]
real_to_part = np.array(meta["real_to_part"])  # fine real -> padded fine pos
part_to_real = np.array(meta["part_to_real"])  # padded fine pos -> fine real
going_next   = np.array(meta["going_next"])    # padded pos at level X -> padded pos at level X+1

print(f"N(fine)={N} BS={BS} LEV={LEV} PAD={PAD}")
for L in levels:
    print(f"  L{L['level']}: count={L['count']} offset={L['offset']}")

# --- load fine BCOO ---
trips = []
with open(DBG / "abd_bcoo.f1.n0.txt") as f:
    for line in f:
        if line.startswith("#"): continue
        s = line.strip().split()
        if len(s) < 11: continue
        r, c = int(s[0]), int(s[1])
        v = np.array([float(x) for x in s[2:11]]).reshape(3, 3)
        trips.append((r, c, v))
print(f"BCOO triplets: {len(trips)}")

# Build dense fine H_0 (3N x 3N) symmetrically
H = np.zeros((3*N, 3*N))
for r, c, v in trips:
    H[3*r:3*r+3, 3*c:3*c+3] += v
    if r != c:
        H[3*c:3*c+3, 3*r:3*r+3] += v.T
print(f"Built H ({H.shape}), |H|_F = {np.linalg.norm(H):.4e}, sym err = {np.linalg.norm(H - H.T):.4e}")

# --- Build R_1 (sum aggregation) using going_next on padded fine positions ---
# For each fine real node a in [0, N), its padded fine pos = real_to_part[a].
# Its L1 padded position = going_next[real_to_part[a]].
# The L1 real index in [0, count(L1)) is L1_padded - offset(L1).
L1 = next(L for L in levels if L["level"] == 1)
N1, OFF1 = L1["count"], L1["offset"]

R1 = lil_matrix((3*N1, 3*N))
# Note: at L0 layer, going_next is indexed by REAL fine index (not padded position)
# because build_level1 writes going_next_L0(part_to_real(tid)) = ...
for a in range(N):
    p1 = going_next[a]
    if p1 < 0: continue
    coarse_idx = p1 - OFF1
    if not (0 <= coarse_idx < N1): continue
    for k in range(3):
        R1[3*coarse_idx + k, 3*a + k] = 1.0
R1 = R1.tocsr()
print(f"Built R_1 ({R1.shape}), nnz={R1.nnz}")

# Galerkin coarse Hessian
H1_galerkin = (R1 @ H @ R1.T).toarray() if False else (R1 @ (H @ R1.T.toarray()))
print(f"Built Galerkin H_1, shape={H1_galerkin.shape}, |H_1|_F = {np.linalg.norm(H1_galerkin):.4e}")

# --- Load MAS-built L1 cluster matrices (from .mtx) and reconstruct full H_1 dense ---
NUM_BLOCKS = PAD // BS
DIM = BS * 3
print(f"Cluster bank count = {NUM_BLOCKS}")

clusters = [np.zeros((DIM, DIM)) for _ in range(NUM_BLOCKS)]
max_idx = 0
with open(DBG / "mas_cluster_hess.f1.n0.mtx") as f:
    for line in f:
        if line.startswith('%'): continue
        s = line.strip().split()
        if len(s) < 3: continue
        try:
            i, j, v = int(s[0]) - 1, int(s[1]) - 1, float(s[2])
        except ValueError:
            continue
        max_idx = max(max_idx, i, j)
        cid_i = i // DIM
        cid_j = j // DIM
        if cid_i != cid_j:
            continue
        if cid_i >= NUM_BLOCKS:
            continue
        ii, jj = i % DIM, j % DIM
        clusters[cid_i][ii, jj] += v
print(f"  mtx max index = {max_idx} (NUM_BLOCKS*DIM = {NUM_BLOCKS*DIM})")

# Symmetrize block-wise: for off-diag blocks (br != bc), only upper is stored; mirror to lower.
# Diagonal blocks (br == bc) are stored as full 3x3 (already symmetric).
for c in range(NUM_BLOCKS):
    M = clusters[c]
    M_sym = np.zeros_like(M)
    for br in range(BS):
        for bc in range(br, BS):
            blk = M[br*3:br*3+3, bc*3:bc*3+3]
            M_sym[br*3:br*3+3, bc*3:bc*3+3] = blk
            if bc > br:
                M_sym[bc*3:bc*3+3, br*3:br*3+3] = blk.T
    clusters[c] = M_sym

# Reconstruct H_1_actual on the padded L1 layout, then extract real positions
# L1 padded positions are [OFF1, OFF1+pad_count_L1). Each L1 bank is BS positions.
# Bank b in L1 corresponds to cluster index (OFF1//BS) + b in cluster_hess array.
L1_BANK_BASE = OFF1 // BS
L1_PAD_NODES = (next(L for L in levels if L["level"] == 2)["offset"] - OFF1)  # padded count
L1_NUM_BANKS = L1_PAD_NODES // BS
print(f"L1 banks: {L1_NUM_BANKS}, padded L1 nodes: {L1_PAD_NODES}")

H1_actual_pad = np.zeros((3 * L1_PAD_NODES, 3 * L1_PAD_NODES))
for b in range(L1_NUM_BANKS):
    cid = L1_BANK_BASE + b
    H1_actual_pad[3*b*BS:3*(b+1)*BS, 3*b*BS:3*(b+1)*BS] = clusters[cid]

# Extract real L1 positions: padded pos OFF1 + i for i in [0, L1_PAD_NODES). The real ones are i < N1.
H1_actual = H1_actual_pad[:3*N1, :3*N1]
print(f"|H_1_actual|_F = {np.linalg.norm(H1_actual):.4e}")
print(f"|H_1_galerkin - H_1_actual|_F = {np.linalg.norm(H1_galerkin - H1_actual):.4e}")
print(f"  rel = {np.linalg.norm(H1_galerkin - H1_actual) / (np.linalg.norm(H1_galerkin) + 1e-30):.4e}")

# Per-block diff
print("\nPer-block max abs diff (top 10 worst):")
diffs = []
for br in range(N1):
    for bc in range(br, N1):
        blk_g = H1_galerkin[3*br:3*br+3, 3*bc:3*bc+3]
        blk_a = H1_actual[3*br:3*br+3, 3*bc:3*bc+3]
        d = np.abs(blk_g - blk_a).max()
        if d > 1e-6:
            diffs.append((d, br, bc, blk_g, blk_a))
diffs.sort(reverse=True)
for d, br, bc, bg, ba in diffs[:10]:
    print(f"  ({br},{bc}) maxdiff={d:.4e}  galerkin_norm={np.linalg.norm(bg):.4e}  actual_norm={np.linalg.norm(ba):.4e}")

# ---- Direct check: sum of cluster_hess[19] vs Galerkin H_1[19,19] ----
print("\n=== Direct L1 diag check for fine cluster bank 19 (L1 padded 419) ===")
# Find the L0 bank that maps to L1 padded 419
# L0 bank b contains real positions [b*16, b*16+16). For these positions p,
# part_to_real[p] gives real fine index. going_next[real fine] gives L1 padded.
target_L1pad = 419
fine_real_in_target = [a for a in range(N) if going_next[a] == target_L1pad]
print(f"Fine real nodes -> L1 padded {target_L1pad}: {fine_real_in_target[:5]}...{fine_real_in_target[-2:]}")
padded_in_target = sorted(int(real_to_part[a]) for a in fine_real_in_target)
print(f"Padded positions: {padded_in_target}")
fine_bank_id = padded_in_target[0] // BS
print(f"Fine bank id = {fine_bank_id}")

# Sum all 256 entries of cluster_hess[fine_bank_id]
M = clusters[fine_bank_id]  # 48x48
block_sum = np.zeros((3,3))
for br in range(BS):
    for bc in range(BS):
        block_sum += M[br*3:br*3+3, bc*3:bc*3+3]
print(f"Sum of all 256 3x3 blocks of cluster_hess[{fine_bank_id}]:\n{block_sum}")
print(f"Frobenius norm = {np.linalg.norm(block_sum):.4e}")

gal_block = H1_galerkin[3*19:3*19+3, 3*19:3*19+3]
print(f"\nGalerkin H_1[19,19] block:\n{gal_block}")
print(f"Frobenius norm = {np.linalg.norm(gal_block):.4e}")

cluster_L1 = clusters[26]
actual_diag = cluster_L1[3*3:3*3+3, 3*3:3*3+3]
print(f"\nActual cluster_hess[26][9:12,9:12] (= L1 diag at padded 419):\n{actual_diag}")
print(f"Frobenius norm = {np.linalg.norm(actual_diag):.4e}")
print(f"\nExtra (actual - block_sum):\n{actual_diag - block_sum}")
print(f"|extra|_F = {np.linalg.norm(actual_diag - block_sum):.4e}")

print("\n=== Raw L1 cluster 26 (padded 419) diag block (3,3) - check asymmetry ===")
raw26_33 = np.zeros((3, 3))
with open(DBG / "mas_cluster_hess.f1.n0.mtx") as f:
    for line in f:
        if line.startswith('%'): continue
        s = line.strip().split()
        if len(s) < 3: continue
        try:
            i, j, v = int(s[0]) - 1, int(s[1]) - 1, float(s[2])
        except ValueError:
            continue
        if i // DIM == 26 and j // DIM == 26:
            ii, jj = i % DIM, j % DIM
            br, bc = ii // 3, jj // 3
            if br == 3 and bc == 3:
                raw26_33[ii % 3, jj % 3] = v
print(f"raw cluster_hess[26].M[sym(3,3)]:\n{raw26_33}")
print(f"  asym (M - M.T):\n{raw26_33 - raw26_33.T}")
print(f"  |asym|_F = {np.linalg.norm(raw26_33 - raw26_33.T):.4e}")

print("\n=== Asymmetry of fine cluster 19's diagonal 3x3 blocks ===")
# Read RAW (un-symmetrized) cluster_hess for fine cluster 19 from .mtx
# The clusters[] list above was symmetrized. Let me re-read raw here.
raw19 = np.zeros((BS*3, BS*3))
with open(DBG / "mas_cluster_hess.f1.n0.mtx") as f:
    for line in f:
        if line.startswith('%'): continue
        s = line.strip().split()
        if len(s) < 3: continue
        try:
            i, j, v = int(s[0]) - 1, int(s[1]) - 1, float(s[2])
        except ValueError:
            continue
        if i // DIM == 19 and j // DIM == 19:
            raw19[i % DIM, j % DIM] = v

asym_total = np.zeros((3,3))
for lr in range(BS):
    blk = raw19[lr*3:lr*3+3, lr*3:lr*3+3]
    asym = blk - blk.T
    if np.linalg.norm(asym) > 1e-6:
        print(f"  diag block lr={lr}: |asym|={np.linalg.norm(asym):.4e}")
        print(f"    blk = {blk}")
    asym_total += asym
print(f"  total asym sum = \n{asym_total}")
print(f"  |total asym|_F = {np.linalg.norm(asym_total):.4e}")
print(f"\nExpected: Galerkin diag should == sum of fine cluster intra (block_sum). They match? {np.allclose(block_sum, gal_block, rtol=1e-3)}")
print(f"Expected: Actual L1 diag should == block_sum (Pass 2 prefix==1 sum). They match? {np.allclose(block_sum, actual_diag, rtol=1e-3)}")

# ---- L2 verification ----
print("\n=== L2 verification ===")
L2 = next(L for L in levels if L["level"] == 2)
N2, OFF2 = L2["count"], L2["offset"]
L2_PAD_NODES = (next(L for L in levels if L["level"] == 3)["offset"] - OFF2)
L2_NUM_BANKS = L2_PAD_NODES // BS
print(f"L2 real={N2} padded={L2_PAD_NODES} banks={L2_NUM_BANKS}")

# R_2: fine real -> L2 padded position (via L0 -> L1 -> L2)
R2 = lil_matrix((3*N2, 3*N))
for a in range(N):
    p1 = going_next[a]            # L0 -> L1 padded
    if p1 < 0: continue
    p2 = going_next[p1]            # L1 padded -> L2 padded
    if p2 < 0: continue
    coarse_idx = p2 - OFF2
    if not (0 <= coarse_idx < N2): continue
    for k in range(3):
        R2[3*coarse_idx + k, 3*a + k] = 1.0
R2 = R2.tocsr()
H2_galerkin = R2 @ (H @ R2.T.toarray())
print(f"|H_2 galerkin|_F = {np.linalg.norm(H2_galerkin):.4e}")

L2_BANK_BASE = OFF2 // BS
H2_actual_pad = np.zeros((3*L2_PAD_NODES, 3*L2_PAD_NODES))
for b in range(L2_NUM_BANKS):
    cid = L2_BANK_BASE + b
    if cid < NUM_BLOCKS:
        H2_actual_pad[3*b*BS:3*(b+1)*BS, 3*b*BS:3*(b+1)*BS] = clusters[cid]
H2_actual = H2_actual_pad[:3*N2, :3*N2]
print(f"|H_2 actual  |_F = {np.linalg.norm(H2_actual):.4e}")
print(f"|H_2 diff    |_F = {np.linalg.norm(H2_galerkin - H2_actual):.4e}")
print(f"  rel = {np.linalg.norm(H2_galerkin - H2_actual)/(np.linalg.norm(H2_galerkin)+1e-30):.4e}")
if np.linalg.norm(H2_galerkin - H2_actual) / np.linalg.norm(H2_galerkin) > 1e-3:
    print("H_2 galerkin (block norms) -- significant divergence detected:")
    for i in range(N2):
        for j in range(N2):
            bg = H2_galerkin[3*i:3*i+3, 3*j:3*j+3]
            ba = H2_actual[3*i:3*i+3, 3*j:3*j+3]
            print(f"  ({i},{j}) g={np.linalg.norm(bg):.3e} a={np.linalg.norm(ba):.3e}")

# ---- L3 verification ----
print("\n=== L3 verification ===")
try:
    L3 = next(L for L in levels if L["level"] == 3)
    N3, OFF3 = L3["count"], L3["offset"]
    L3_PAD_NODES = (next(L for L in levels if L["level"] == 4)["offset"] - OFF3)
    L3_NUM_BANKS = L3_PAD_NODES // BS
    print(f"L3 real={N3} padded={L3_PAD_NODES} banks={L3_NUM_BANKS}")
    R3 = lil_matrix((3*N3, 3*N))
    for a in range(N):
        p1 = going_next[a]
        if p1 < 0: continue
        p2 = going_next[p1]
        if p2 < 0: continue
        p3 = going_next[p2]
        if p3 < 0: continue
        g = p3 - OFF3
        if not (0 <= g < N3): continue
        for k in range(3):
            R3[3*g + k, 3*a + k] = 1.0
    R3 = R3.tocsr()
    H3_galerkin = R3 @ (H @ R3.T.toarray())
    print(f"|H_3 galerkin|_F = {np.linalg.norm(H3_galerkin):.4e}")

    L3_BANK_BASE = OFF3 // BS
    H3_actual_pad = np.zeros((3*L3_PAD_NODES, 3*L3_PAD_NODES))
    for b in range(L3_NUM_BANKS):
        cid = L3_BANK_BASE + b
        if cid < NUM_BLOCKS:
            H3_actual_pad[3*b*BS:3*(b+1)*BS, 3*b*BS:3*(b+1)*BS] = clusters[cid]
    H3_actual = H3_actual_pad[:3*N3, :3*N3]
    print(f"|H_3 actual  |_F = {np.linalg.norm(H3_actual):.4e}")
    print(f"|H_3 diff    |_F = {np.linalg.norm(H3_galerkin - H3_actual):.4e}")
    if np.linalg.norm(H3_galerkin) > 0:
        print(f"  rel = {np.linalg.norm(H3_galerkin - H3_actual)/np.linalg.norm(H3_galerkin):.4e}")
except StopIteration:
    print("L3 not present")

# ---- L2 detailed analysis: explicit Pass1 + Pass2 simulation ----
print("\n=== L2 explicit reconstruction ===")
# For each L2 group g (coarse_idx), find which fine real nodes map to it.
L2_groups = [[] for _ in range(N2)]
for a in range(N):
    p1 = going_next[a]
    if p1 < 0: continue
    p2 = going_next[p1]
    if p2 < 0: continue
    g = p2 - OFF2
    if 0 <= g < N2:
        L2_groups[g].append(a)
for g in range(N2):
    print(f"  L2 group {g}: {len(L2_groups[g])} fine nodes, range [{min(L2_groups[g])}, {max(L2_groups[g])}]")

# Group fine nodes by fine cluster bank
fine_bank_of = {}
for a in range(N):
    p = real_to_part[a]
    if p >= 0:
        fine_bank_of[a] = p // BS

# For each L2 group, identify fine clusters mapping to it
fine_clusters_in_L2grp = [set() for _ in range(N2)]
for g in range(N2):
    for a in L2_groups[g]:
        fine_clusters_in_L2grp[g].add(fine_bank_of[a])
for g in range(N2):
    print(f"  L2 group {g} fine clusters: {sorted(fine_clusters_in_L2grp[g])}")

# Compute Pass 2 contribution: sum of block_sum_X for each X in L2 group g
def block_sum_of(c):
    M = clusters[c]
    s = np.zeros((3,3))
    for br in range(BS):
        for bc in range(BS):
            s += M[br*3:br*3+3, bc*3:bc*3+3]
    return s

print("\n=== Per L2-group breakdown for diag (0,0) block ===")
for g in [0]:
    pass2_sum = np.zeros((3,3))
    for X in fine_clusters_in_L2grp[g]:
        bs = block_sum_of(X)
        pass2_sum += bs
        print(f"  fine cluster {X} block_sum norm = {np.linalg.norm(bs):.4e}")
    print(f"  Pass2 total = {np.linalg.norm(pass2_sum):.4e}")

    # Pass 1 contribution: cross-fine entries with both endpoints in L2 group g
    pass1_sum = np.zeros((3,3))
    for r, c, v in trips:
        if r == c: continue
        ra, ca = (r in L2_groups[g]), (c in L2_groups[g])
        rb, cb = fine_bank_of.get(r), fine_bank_of.get(c)
        if ra and ca and rb != cb:
            pass1_sum += 2 * v  # diag-doubling at L2 (vert_col == vert_row)
    print(f"  Pass1 cross-fine to (g,g) total = {np.linalg.norm(pass1_sum):.4e}")
    print(f"  Total expected (Pass1+Pass2) = {np.linalg.norm(pass1_sum + pass2_sum):.4e}")
    print(f"  Galerkin L2[{g},{g}] = {np.linalg.norm(H2_galerkin[3*g:3*g+3, 3*g:3*g+3]):.4e}")
    print(f"  Actual L2[{g},{g}] = {np.linalg.norm(H2_actual[3*g:3*g+3, 3*g:3*g+3]):.4e}")
    print(f"  expected - actual diff = {np.linalg.norm(pass1_sum + pass2_sum - H2_actual[3*g:3*g+3, 3*g:3*g+3]):.4e}")

# ---- diagnostic: how many L1 banks ----
print(f"\nL1 mapping stats:")
counts = np.zeros(L1_PAD_NODES, dtype=int)
for a in range(N):
    p1 = going_next[a]
    if p1 >= 0:
        counts[p1 - OFF1] += 1
print(f"  fine nodes per L1 node: min={counts[counts>0].min()}, max={counts.max()}, mean={counts[counts>0].mean():.1f}")
print(f"  L1 nodes used: {(counts > 0).sum()} / {L1_PAD_NODES}")
