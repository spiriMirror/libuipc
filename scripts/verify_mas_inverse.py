"""Verify cluster inverses are correct."""
import numpy as np
import json
import sys
from pathlib import Path

DBG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\81946\Projects\LibuipcWithAssets\libuipc\output\tests\sim_case\93_abd_mas_vs_diag_benchmark.cpp\mas\debug\cuda\affine_body\abd_mas_preconditioner.cu")
print(f"Loading: {DBG}")
meta = json.load(open(DBG / "mas_cluster_meta.f1.n0.json"))
PAD = meta["total_clusters"]
BS = meta["banksize"]
NUM_BLOCKS = PAD // BS
DIM = BS * 3
levels = meta["levels"]

def load_mtx(path):
    arr = [np.zeros((DIM, DIM)) for _ in range(NUM_BLOCKS)]
    with open(path) as f:
        for line in f:
            if line.startswith("%"):
                continue
            s = line.strip().split()
            if len(s) < 3:
                continue
            try:
                i, j, v = int(s[0]) - 1, int(s[1]) - 1, float(s[2])
            except ValueError:
                continue
            cid = i // DIM
            if cid != j // DIM or cid >= NUM_BLOCKS:
                continue
            arr[cid][i % DIM, j % DIM] = v
    # symmetrize block-wise
    for c in range(NUM_BLOCKS):
        M = arr[c]
        Ms = np.zeros_like(M)
        for br in range(BS):
            for bc in range(br, BS):
                blk = M[br * 3:br * 3 + 3, bc * 3:bc * 3 + 3]
                Ms[br * 3:br * 3 + 3, bc * 3:bc * 3 + 3] = blk
                if bc > br:
                    Ms[bc * 3:bc * 3 + 3, br * 3:br * 3 + 3] = blk.T
        arr[c] = Ms
    return arr

hess = load_mtx(DBG / "mas_cluster_hess.f1.n0.mtx")
inv  = load_mtx(DBG / "mas_cluster_inv.f1.n0.mtx")

print("\n=== cluster inverse verification ===")
for L in levels:
    OFF = L["offset"]
    bank = OFF // BS
    if bank >= NUM_BLOCKS:
        continue
    A = hess[bank]
    Ai = inv[bank]
    # mirror identity-pad: replace zero diagonals with 1.0 (matches GJ inverse hack)
    A_pad = A.copy()
    for i in range(DIM):
        if A_pad[i, i] == 0:
            A_pad[i, i] = 1.0
    AiA = Ai @ A_pad
    err = np.max(np.abs(AiA - np.eye(DIM)))
    cond = np.linalg.cond(A_pad)
    Ai_np = np.linalg.inv(A_pad)
    inv_err = np.max(np.abs(Ai - Ai_np))
    print(f"  L{L['level']} cluster {bank} ({L['count']} real nodes):  "
          f"|A_inv_gpu * A - I|_max = {err:.4e}, "
          f"|A_inv_gpu - A_inv_numpy|_max = {inv_err:.4e}, "
          f"cond(A_pad) = {cond:.4e}")
