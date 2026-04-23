"""
Analyze ABD MAS cluster matrices from the .mtx debug dumps.

The MTX is in Matrix Market coordinate format:
  row col val  (1-indexed, upper-triangle 3x3 blocks per cluster, block-diagonal layout)
Each cluster c occupies rows/cols [c*48+1 .. c*48+48] (48 = BANKSIZE*3).
"""

import sys, json, pathlib
import numpy as np
import scipy.io as sio

BANKSIZE = 16
DIM = BANKSIZE * 3  # 48 DOFs per cluster

def load_cluster_mats(mtx_path, num_cluster_blocks):
    """
    Load cluster matrices from COO MTX. Returns list of dense (48x48) numpy arrays
    (one per cluster block). Symmetrizes the upper-triangle.
    """
    m = sio.mmread(str(mtx_path))  # returns sparse COO or dense
    # Convert to dense
    try:
        A = m.toarray()
    except AttributeError:
        A = np.array(m)

    clusters = []
    for c in range(num_cluster_blocks):
        lo = c * DIM
        hi = lo + DIM
        blk = A[lo:hi, lo:hi].copy()
        # Symmetrize (dump stores upper triangle only)
        blk = blk + blk.T - np.diag(np.diag(blk))
        clusters.append(blk)
    return clusters

def analyze(hess_path, inv_path, meta_path):
    with open(meta_path) as f:
        meta = json.load(f)

    num_cluster_blocks = meta["num_cluster_blocks"]
    total_clusters     = meta["total_clusters"]
    print(f"Meta: num_cluster_blocks={num_cluster_blocks}, total_clusters={total_clusters}")
    print(f"  (fine-level clusters = total_map_nodes/BANKSIZE = {meta['total_map_nodes']//BANKSIZE})")
    print()

    hess_clusters = load_cluster_mats(hess_path, num_cluster_blocks)
    inv_clusters  = load_cluster_mats(inv_path,  num_cluster_blocks)

    issues = 0
    fine_cluster_count = meta['total_map_nodes'] // BANKSIZE  # = 25 for 100 bodies

    print(f"{'Cluster':>8} {'Level':>6} {'eig_min':>12} {'eig_max':>12} {'cond':>10} {'PH_err':>10} {'Notes'}")
    print("-" * 75)
    for ci in range(num_cluster_blocks):
        H = hess_clusters[ci]
        P = inv_clusters[ci]

        # Which level is this cluster?
        base_idx = ci * BANKSIZE
        if base_idx < meta['total_map_nodes']:
            level = 0
        elif base_idx < meta['total_map_nodes'] + 32:
            level = 1
        elif base_idx < meta['total_map_nodes'] + 32 + 16:
            level = 2
        else:
            level = 3

        # Check eigenvalues of H
        eigvals = np.linalg.eigvalsh(H)
        min_eig = eigvals[0]
        max_eig = eigvals[-1]
        cond    = max_eig / max(abs(min_eig), 1e-300)

        # Check P ≈ H^{-1}: ||P H - I||_inf
        PH  = P @ H
        I   = np.eye(DIM)
        err = np.max(np.abs(PH - I))

        notes = ""
        if min_eig <= 1e-10:
            notes += f" SINGULAR(eig_min={min_eig:.2e})"
            issues += 1
        elif min_eig < 0:
            notes += f" NOT_PD"
            issues += 1
        if err > 0.05:
            notes += f" BAD_INV"
            issues += 1

        # Print all fine-level clusters and any problematic ones
        if ci < fine_cluster_count or notes or ci == fine_cluster_count:
            print(f"{ci:8d} {'L'+str(level):>6} {min_eig:12.4e} {max_eig:12.4e} {cond:10.2e} {err:10.4f}{notes}")

    print()
    print(f"Total cluster blocks: {num_cluster_blocks}, issues: {issues}")

    # Special: check if body-0 (fixed) cluster is correct
    print()
    print("=== Cluster 0 (bodies 0-3, body 0 is fixed) diagonal 3x3 blocks ===")
    H0 = hess_clusters[0]
    for node in range(4):
        lo = node * 3; hi = lo + 3
        diag_blk = H0[lo:hi, lo:hi]
        eig = np.linalg.eigvalsh(diag_blk)
        print(f"  Node {node} (body {node//4}, body-node {node%4}): diag block eigs = {eig}")

if __name__ == "__main__":
    base = pathlib.Path(
        r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case"
        r"\93_abd_mas_vs_diag_benchmark.cpp\mas\debug\cuda\affine_body\abd_mas_preconditioner.cu"
    )
    hess = base / "mas_cluster_hess.f1.n0.mtx"
    inv  = base / "mas_cluster_inv.f1.n0.mtx"
    meta = base / "mas_cluster_meta.f1.n0.json"

    if len(sys.argv) == 4:
        hess, inv, meta = map(pathlib.Path, sys.argv[1:4])

    analyze(str(hess), str(inv), str(meta))
