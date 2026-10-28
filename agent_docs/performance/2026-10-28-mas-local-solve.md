# 2026-10-28 — MAS Local Solve Output Writes

- Status: Accepted scoped optimization; unrelated baseline test failure retained
- Before: `c20a5aa4033cf8eceeb49f3800594a76dd782cdb`
- After: the commit containing this record
- Samples: `77f43414b649ad6c94387f367f2d0edabffa8a9f`
- Scope: remove redundant work in MAS application; no material, solver
  tolerance, partition, precision, scene or public API changes

## Question and profiling

Can MAS apply avoid clearing a buffer and atomically adding results when each
output row has a single writer?

Fresh synchronized stage profiles identified linear solving as a dominant
cost. Across 100 frames, the MAS bunny spent 36.47 ms/frame in FusedPCG out of
77.77 ms/frame in the simulation; case2 spent 45.18 out of 222.61 ms/frame.
These synchronized diagnostic times are not throughput measurements.

Nsight Compute reported `ERR_NVGPUCTRPERM`; kernel performance counters were
unavailable. The installed Nsight Systems also failed to capture CUDA kernels.
The scoped measurements below use CUDA events around graph replay instead.
They establish elapsed cost, not hardware-counter attribution.

## Change and correctness argument

`MASPreconditionerEngine_schwarz_local_solve_kernel` launches one 256-thread
block per 16-node cluster. Each logical group of 16 threads reduces one matrix
row, and only its leader writes that row's `float3`. Every allocated active
cluster row, including padding, is covered by exactly one such group.

Replace the three `atomicAdd` operations with a direct `float3` assignment and
remove the preceding `multi_level_Z` zero-fill. This removes one kernel launch
per preconditioner application and three atomics per row. The CUB reduction,
matrix arithmetic and mixed-precision storage are unchanged. Coarse residual
restriction still has multiple writers and retains its zero-fill and atomics.

When convergence is already set, both the local solve and subsequent
prolongation skip their work; no consumer observes the skipped intermediate
buffer. The next unconverged application overwrites every row before the
prolongation kernel reads it on the same stream.

## Measurement environment

This describes one local experiment, not required community hardware or a
performance gate. Use fresh same-machine measurements on other systems.

| Field | Value |
|---|---|
| CPU / GPU | Core i9-12900K / GeForce RTX 4090 (display device) |
| OS / driver | Windows 10 19045 / NVIDIA 591.44 |
| Compiler / CUDA | MSVC 19.39 / CUDA 12.4.131 |
| Build / Python | Release, `sm_89`, CPython 3.12.7 |
| Before backend SHA-256 | `6c600b417986df0b2898b2c4a4345def8cae37ce58842f75dffd28024e1e77a9` |
| After backend SHA-256 | `30c01f5ea7117645496f5420b7138ee5bee0f9ba49695a31529a0569cc8ae251` |

The initial Release build was up to date and matched the installed runtime.
Frozen package copies allowed before/after subprocesses to alternate without
rebuilding between measurements. The candidate contained only the source
change described above. Both input and runtime pre/post fingerprints were
stable in every run; samples and manifest parameters were unchanged.

## Scoped measurement and validation

An ignored probe extracts the actual local-solve kernels from the before and
after sources. It compares the old clear-plus-solve path with the new direct
solve using the same symmetric matrices and residuals. CUDA events measure
200 operations per graph replay, alternating variants for six rounds after
warming both graphs. Initialization and host submission loops are excluded.

| Clusters | Before median (microseconds/apply) | After median | Change |
|---:|---:|---:|---:|
| 3 | 2.5984 | 1.8432 | -29.1% |
| 31 | 2.6290 | 1.8450 | -29.8% |
| 128 | 2.8181 | 2.0173 | -28.4% |
| 512 | 3.7989 | 2.9286 | -22.9% |

The one-cluster probe was noisier (2.4781 to 2.2554 microseconds, -9.0%). These
are measurements of the **clear-plus-local-solve path**, not the entire MAS
preconditioner or simulation. Savings cannot be transferred directly to frame
time.

For 1, 3, 31, 128 and 512 clusters, both implementations produced equal finite
results. An independent double-precision dense matrix-vector reference had a
maximum absolute error of `1.394e-6`, consistent with float storage/arithmetic.
Seeding the new output with NaNs checked complete overwrite; a convergence
flag check verified that sentinel outputs remained untouched when skipped.
Compute Sanitizer memcheck reported zero errors; racecheck reported zero
hazards, errors or warnings on these probes.

## End-to-end validation

Three fresh processes per version, alternating before/after order, use each
manifest entry's full default frame window. All 24 runs completed and converged
without iteration-limit hits. Values below are medians of run means; ranges
show the three individual run means.

| Workload | Frames | Before ms/frame (range) | After ms/frame (range) | Change |
|---|---:|---:|---:|---:|
| MAS bunny | 100 | 75.065 (74.924–75.159) | 74.650 (74.593–75.081) | -0.55% |
| FEM/cloth case2 | 250 | 230.734 (228.861–232.862) | 230.406 (229.869–231.547) | -0.14% |
| ABD wall / cloth | 100 | 123.202 (120.019–126.185) | 123.299 (121.165–124.097) | +0.08% |
| Pure ABD control | 120 | 156.982 (155.071–157.444) | 158.379 (154.889–158.446) | +0.89% |

MAS bunny improved in each paired run, with unchanged Newton iterations/frame
(4.86) and essentially unchanged linear iterations/frame (386.683 vs 386.700).
Its final centroid differed by a few micrometers. The case2 and wall differences
are smaller than repeat variation; they do not establish an end-to-end speedup.
The pure ABD scene does not invoke MAS and serves as a variability control.

Collision-sensitive final observables are not bitwise goldens. Case2's final
upper-bunny x centroid ranged from -1.0958 to -1.0931 before and -1.1096 to
-1.0933 after; the lower bunny and the unchanged ABD control also varied between
runs. Three repetitions do not establish a full trajectory envelope. Exact
local-solve equivalence and the regression suite provide correctness evidence;
the trajectory ranges alone are not proof of either a defect or equivalence.

Separate paired stage diagnostics (three 100-frame runs per version) measured
the enclosing MAS-bunny FusedPCG scope at 35.5945 to 35.1128 ms/frame (-1.35%).
The run ranges were 35.4356–35.7791 before and 34.9570–35.3431 after. The full
linear-system scope moved from 57.4623 to 57.0227 ms/frame (-0.77%). This is
consistent with removing repeated work inside PCG; synchronized stage numbers
must not be mixed with timer-free throughput.

Native validation:

- CPU CTest groups `common`, `core` and `geometry`: passed.
- CUDA backend: 27 cases / 805 assertions passed; sanity checking: 3 cases /
  100 assertions passed; regression: 1 case / 4 assertions passed.
- Full single-process simulation aggregate: 94 of 95 cases passed, 14211 of
  14213 assertions passed. The two failures are both in
  `86_discrete_shell_bending_stress_plastic_residual`, which requires a plastic
  residual angle above 0.05 rad; the observed value was approximately 0.03314.
- A second full aggregate using the frozen **before** CUDA backend reproduced
  exactly the same failing case and assertion counts. The isolated baseline
  case also failed at 0.0331428 rad, versus 0.0331431 in the candidate aggregate.
  This test uses the default diagonal preconditioner, not MAS. It is a separate
  existing test/model investigation; neither its thresholds nor constitutions
  were changed to make this optimization appear fully green.

The aggregate is therefore **not fully passing**, but the optimization
introduced no additional observed failures. Existing MAS simulations, including
custom partitions, cloth/rod cases, Empty fallback and the non-graph comparison,
passed as part of the aggregate.

Release packaging preserved the uninstall-first policy. The optimized package
was installed in the build environment and default CPython 3.12 interpreter;
installed MAS-bunny and small-cloth GPU smoke runs passed. The backend hash
matches the candidate above. All 67 repository script tests, registry input
validation, API parity, workflow pins, zero-byte checks and clang-format 18
checks passed.

## Decision

Accept the narrow change on the strength of exact local-result comparisons,
sanitizer checks, the enclosing PCG measurements and small MAS-bunny throughput
benefit, with no new observed native test failures. It is not a broad frame-rate
improvement. Keep the pre-existing bending-plasticity failure visible; larger
optimizations require their own profiles and controlled before/after evidence.

## Rejected alternatives

Padded shared-memory rows, fewer pivot synchronization phases and a single
cluster per block were explored in isolated inversion probes. They did not
establish a consistent gain across cluster counts and were not applied to
production code. The original 48x48 inversion kernel remains unchanged.

## Reproduction and artifacts

Use the versioned runner at the before and after revisions, with corresponding
Release runtime packages, on the same machine. Run each command three times
per version in alternating order:

```shell
python scripts/run_benchmark.py run mas-bunny
python scripts/run_benchmark.py run stiff-gipc-case2
python scripts/run_benchmark.py run cube-wall-cloth
python scripts/run_benchmark.py run rigid-wrecking-balls
```

Keep `UIPC_BENCHMARK_TIMERS=0` for throughput. Separate stage profiles use
`uipc.profile.run` on the initialized sample world, with timers enabled and
100 measured frames. Do not compare those synchronized values to throughput.

Ignored local evidence is under `output/perf-2026-10-28/`: `profile-before/`
(including `timer_frames.json`), `runtime-before/`, `runtime-after/`,
`exploratory.json` (60-frame screening), `full.json`, `stage-{before,after}-*/`,
`local-probe.log`,
`local-{memcheck,racecheck}.log`, source-extraction probes and build logs.
Native validation logs include `ctest-{fast,gpu}.log`,
`baseline-sim-all.{xml,log}` and `baseline-bending86.xml`.
Raw timestamped runner metadata and logs remain in `output/benchmark-runs/`.
