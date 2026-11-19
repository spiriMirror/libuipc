# Performance Evidence

This directory stores durable, reviewable performance conclusions. The
chronological `handoff.md` retains raw session history; accepted/rejected
experiments should gain a focused record here so future agents can distinguish
measurement from intuition.

## Evidence hierarchy

1. Correctness and safety gates first: focused tests, full aggregate simulation,
   and sanitizer coverage appropriate to the change.
2. Measure the narrow kernel/scope that the change targets.
3. Measure the enclosing subsystem and end-to-end benchmark.
4. Repeat noisy wall measurements and report variation; do not turn one run
   into a claim.

Kernel resource counts, launch counts, or attractive assembly alone do not prove
a speedup. Conversely, contact-sensitive wall time can hide a real scoped gain
when iteration counts change. Report both and state which is the primary signal.

## Community benchmark policy

Version the workloads, metrics and reproduction commands. Hardware is result
metadata, not a project-wide required GPU or speed threshold. Use the
[benchmark guide](../../docs/development/testing_and_benchmarks.md) to choose a
small, standard or stress workload and compare versions on the same machine
with fixed inputs and build settings. Ordinary CI checks the pinned samples
contract without a GPU. Any performance gate belongs to a specific runner and
must be calibrated from repeated measurements on that runner.

## Records

| Record | Status | Scope |
|---|---|---|
| [2026-11-19 Linear solve: BCOO conversion, MAS, fused PCG iteration (PR 5/6)](2026-11-19-linear-solve.md) | Proposed | PR 5/6: upper BCOO conversion and SpMV, MAS preconditioner, fused PCG iteration; opt-in keep_preconditioner (off); geometric mean over the five benchmark scenes 1.62x against PR 4 |
| [2026-11-19 FEM assembly beside the contact work (PR 4/6)](2026-11-19-fem-assembly.md) | Proposed | PR 4/6: shell Hessians in shared memory, FEM prelaunch beside DyTopo, in-place FEM assembly, assembly head order; geometric mean over the five benchmark scenes 1.21x against PR 3 |
| [2026-11-19 Contact and affine-body assembly (PR 3/6)](2026-11-19-contact-abd-assembly.md) | Proposed | PR 3/6: fused contact kernels, shared-memory PSD projection, DyTopo distribution, ABD producers on side streams, joint Hessians per warp; geometric mean over the five benchmark scenes 1.48x against PR 2 |
| [2026-11-19 Collision detection: BVH refit, detection reuse, FP32 rejection (PR 2/6)](2026-11-19-collision-detection.md) | Proposed | PR 2/6: LBVH scene-bounds fix, BVH build/refit/queries, reuse across Newton iterations and frames, FP32 rejection, CCD launches; geometric mean over the five benchmark scenes 1.19x against PR 1 |
| [2026-11-19 cuda_tool graph capture, CUB and launch-helper fixes (PR 1/6)](2026-11-19-cuda-tool-capture-cub.md) | Proposed | PR 1/6: graph captures whose body throws, CUB wrappers run once, empty DeviceReduce::Sum, hidden launch helpers; geometric mean over the five benchmark scenes 1.01x against main |
| [2026-10-28 MAS local solve](2026-10-28-mas-local-solve.md) | Accepted scoped optimization | Remove redundant clear/atomics; local path and PCG gains, small end-to-end effect, existing bending-test failure documented |
| [2026-09-01 cross-domain measurements](2026-09-01-cross-domain-baseline.md) | Historical measurement | Three-run ABD, FEM/MAS, cloth/contact envelope on the recorded machine and revisions |
| [2026-08-30 case2 assembly roll-up](2026-08-30-case2-assembly-rollup.md) | Accepted with caveats | Buffer growth, collision readback, line-search aggregation, contact/FEM assembly, rejected split |
| [`handoff.md`](../handoff.md) | Historical source | Earlier MAS, CUDA graph, CUB, DyTopo, lifecycle, and detailed command history |

## Required content

Every new record should identify:

- before/after commits and whether the worktree was clean;
- GPU, driver, CUDA toolkit, build type/flags, and benchmark/submodule revision;
- scene/configuration, warmup, measured frames, repetitions, and timer semantics;
- focused, subsystem, and wall metrics with units and sample counts;
- correctness/sanitizer results and observed numerical variability;
- rejected alternatives and the decision taken;
- artifact paths or commands sufficient to reproduce the run.

Use [0000-evidence-template.md](0000-evidence-template.md). Versioned
scenes are launched through `benchmarks/manifest.json` and
`scripts/run_benchmark.py`; its archived metadata supplies the minimum runtime
and revision facts. For the separate `uipc.profile` runner, approved local
regression thresholds belong in baseline artifacts with environment
compatibility checks. Historical desktop measurements must not be promoted
into another machine's thresholds; capture a fresh baseline on that runner.

## Interpretation rules

- Do not add percentages from different stages as if they were one controlled
  experiment.
- Prefer median plus distribution/repeats for wall time; keep raw means when
  comparing to historical reports that used means.
- Parent timers include children unless explicitly documented otherwise.
- A different Newton/PCG/contact count is an algorithmic-path change, not pure
  throughput evidence.
- Atomic-order trajectory drift must be compared with unchanged-run variability,
  not assumed to be either a bug or harmless.
- Record regressions and rejected designs; they prevent expensive repetition.
