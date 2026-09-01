# 2026-08-30 — Case2 CUDA Assembly Optimization Roll-up

> Historical controlled A/B record. Its 60-frame wall values describe the
> revisions and windows below, not the current main baseline. Use
> [the 2026-09-01 cross-domain baseline](2026-09-01-cross-domain-baseline.md)
> for current end-to-end reference values.

- Status: Accepted with wall-time caveats
- Workload: samples `88_stiff_gipc_benchmark`, MAS, 60 measured frames
- Environment: Windows Release build, NVIDIA RTX 5090, CUDA 13.2
- Commits: `fc5391fc`, `8916b336`, `5931e9ea`, `71f7d53b`

## Question

Could the remaining case2 cost be reduced by removing unnecessary dynamic-buffer
work and host synchronization, then reducing dominant contact/FEM assembly
kernel stack/matrix materialization without changing solver semantics?

## Method and caveats

Nsight Systems and nested libuipc timers identified allocation/energy work,
broad-phase counter readbacks, line-search scalar transfers, and raw
contact/FEM gradient-Hessian assembly. Each stage was measured against its
immediate predecessor; the rows below are not one additive before/after study.
Parent timers include children. Contact ordering changes atomic roundoff and can
change Newton counts, so focused/enclosing scopes are primary and wall values
are supporting evidence.

## Results

| Stage | Focused/enclosing result | Wall evidence |
|---|---|---|
| Discard-aware output growth (`fc5391fc`) | `Scan and Allocate`: 80.8 ms total to 38.0–65.5 ms; two `Compute Energy` scopes: 667.1 ms to 390.2–415.8 ms | 169.8 ms baseline versus 163.7–171.0 ms mean; contact-sensitive |
| Batched collision counts (`8916b336`) | Trajectory detect: 5.07 to 5.01 ms/Newton; aggregate DCD: 4.67 to 4.61 ms/detect | mean/median 163.7/183.2 to 162.7/182.3 ms |
| Device line-search aggregation (`5931e9ea`) | Initial energy: 0.624 to 0.529 ms (-15.2%); trial: 0.542 to 0.448 ms (-17.3%); line search: 7.38 to 7.01 ms/Newton (-5.0%) | mean/median 162.7/182.3 to 158.1/178.4 ms |
| Contact/FEM assembly (`71f7d53b`) | SNH kernel: 1.795 to 1.047 ms (-41.7%); stack 6440 to 1320 B/thread; `Assemble Subsystems`: 3.60 to 2.97 ms/Newton (-17.6%); `Build Linear System`: 7.93 to 7.32 ms/Newton (-7.8%) | two clean runs: 156.0–157.0 ms mean, 173.1–173.9 ms median |

The final contact kernels keep one PT/EE/PE/PP launch with compile-time
gradient-only/Hessian specialization. Gradient-only resource use was 106
registers / 272-byte stack for normal contact and 112 / 144 for friction; the
combined full-Hessian profiler average stayed approximately flat (4.12 to 4.08
ms).

## Rejected alternative

Splitting PT/EE/PE/PP into separate launches reduced static stack use but
serialized rare expensive PT/EE work that previously overlapped the dominant PE
population. `Assemble Dytopo Effect` regressed from 4.52 to 7.67 ms/Newton, so
the split was reverted. Static resource reduction alone was not accepted as
evidence.

## Correctness and numerical behavior

- CUDA backend: 12 cases / 238 assertions after the final assembly stage.
- Focused contact/FEM/MAS/bending: 10 cases / 1426 assertions.
- Full simulation: 95 cases / 14212 assertions.
- Targeted compute-sanitizer: 0 errors and 0 leaked bytes (2 cases / 504
  assertions).
- Atomic-order divergence reached about 0.59 mm by frame 60, within the
  previously observed unchanged-run/baseline-to-change envelope; it was not
  treated as a bitwise determinism claim.

## Decision

Keep discard-aware growth, batched counter/scalar readbacks, direct SNH
projection, and uniform-mode specialization. Keep heterogeneous simplex
Hessian stencils fused until a future end-to-end profile proves a split improves
both the kernel set and enclosing DyTopo scope.

The detailed chronological notes remain in `agent_docs/handoff.md`. Future
reproduction should use `python scripts/run_benchmark.py run stiff-gipc-case2`
so revisions and runtime facts are archived automatically.
