# YYYY-MM-DD — Experiment Title

- Status: Proposed / Accepted / Rejected / Inconclusive
- Before commit:
- After commit:
- Benchmark manifest name and samples commit:

## Question

State the performance hypothesis and the scope expected to move.

## Environment

| Field | Value |
|---|---|
| GPU / driver | |
| CUDA toolkit | |
| OS / compiler | |
| Build type and important flags | |
| Worktree state | clean / dirty (explain) |

## Workload and method

Record scene configuration, warmup, measured frames, repetitions, profiler
commands, and whether timers include children. Describe how before/after order
and thermal effects were controlled.

## Correctness and safety

List focused/full tests, sanitizer results, invariant checks, and numerical
comparison against unchanged-run variability.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | | | | |
| Enclosing subsystem | | | | |
| End-to-end median | | | | |

## Interpretation

Separate directly measured facts from inference. Explain iteration-count or
trajectory differences and identify the primary metric.

## Decision

Accept, reject, revise, or gather more evidence. State the code/config outcome.

## Reproduction and artifacts

List exact commands and ignored artifact paths. Do not commit profiler binaries
or machine-specific output.
