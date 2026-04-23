"""Analyze intra-cluster joint count to measure how much MAS L0 adds over ABDDiag."""
import sys
import re
from pathlib import Path
from collections import defaultdict

p = Path(sys.argv[1])
print(f"Analyzing: {p.parent.parent.parent.parent.parent.parent.name}")

body_cluster = {}
with open(p) as f:
    for line in f:
        m = re.match(r'cluster (\d+)', line)
        if m:
            cid = int(m.group(1))
            for bm in re.finditer(r'b(\d+):', line):
                body_cluster[int(bm.group(1))] = cid
n_bodies = len(body_cluster)
clusters = defaultdict(list)
for b, c in body_cluster.items():
    clusters[c].append(b)
print(f"bodies: {n_bodies}, clusters: {len(clusters)}")

# Try to recover grid dimensions: 4 → 2x2 rows, 16 → 4x4, 900 → 30x30
import math
grid_n = int(round(math.sqrt(n_bodies)))
assert grid_n * grid_n == n_bodies, f"not a square grid? {n_bodies}"
print(f"grid: {grid_n}x{grid_n}")

# Count intra/cross joints
intra = 0
cross = 0
# Horizontal joints (i,j)-(i+1,j)
for j in range(grid_n):
    for i in range(grid_n - 1):
        b1 = i + j * grid_n
        b2 = (i + 1) + j * grid_n
        if body_cluster[b1] == body_cluster[b2]:
            intra += 1
        else:
            cross += 1
# Vertical joints (i,j)-(i,j+1)
for j in range(grid_n - 1):
    for i in range(grid_n):
        b1 = i + j * grid_n
        b2 = i + (j + 1) * grid_n
        if body_cluster[b1] == body_cluster[b2]:
            intra += 1
        else:
            cross += 1

total = intra + cross
print(f"joints: total={total}, intra-cluster={intra} ({100.0*intra/total:.1f}%), cross-cluster={cross} ({100.0*cross/total:.1f}%)")

# Per-cluster joint stats
cluster_intra = defaultdict(int)
for j in range(grid_n):
    for i in range(grid_n - 1):
        b1 = i + j * grid_n
        b2 = (i + 1) + j * grid_n
        if body_cluster[b1] == body_cluster[b2]:
            cluster_intra[body_cluster[b1]] += 1
for j in range(grid_n - 1):
    for i in range(grid_n):
        b1 = i + j * grid_n
        b2 = i + (j + 1) * grid_n
        if body_cluster[b1] == body_cluster[b2]:
            cluster_intra[body_cluster[b1]] += 1
counts = list(cluster_intra.values())
print(f"intra joints per cluster: min={min(counts) if counts else 0}, max={max(counts) if counts else 0}, mean={sum(counts)/len(counts) if counts else 0:.2f}")
# distribution
from collections import Counter
dist = Counter(counts)
print(f"distribution: {dict(sorted(dist.items()))}")
