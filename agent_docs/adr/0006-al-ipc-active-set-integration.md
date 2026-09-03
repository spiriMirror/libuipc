# ADR 0006 — Reimplement AL-IPC Active-Set Improvements Safely

- Status: Accepted
- Date: 2026-09-02
- Owners: CUDA AL-IPC pipeline
- Implements: current AL-IPC integration
- Supersedes: N/A

## Context

The external `wiso-enoji/libuipc` `AL-release` branch adds useful pieces of the
AL-IPC paper to an older libuipc base. Directly cherry-picking it is unsafe: the
current repository has 188 intervening commits, a replacement `cuda_tool`
runtime, explicit dynamic-buffer policies, a richer Newton tolerance manager,
and a single-source scene-config contract.

The fork also performs a 32-bit integer atomic minimum on `Float` storage.
`Float` is `double` here, so that operation addresses only half of an element
and can corrupt the TOI buffer. Its fixed 25-iteration active-pair lifetime does
not implement the paper's `gamma < 0.01` removal rule for arbitrary decay
factors. Its global change from CCD margin 0.1 to 0.001 would alter both IPC and
AL-IPC.

## Decision

Reimplement the algorithm against current interfaces instead of cherry-picking
the fork.

- Exclude existing active pairs, compute the earliest candidate TOI per
  incident vertex, and retain a candidate only when it realizes at least one
  such minimum.
- Use a type-correct 64-bit CAS minimum for double TOIs. Keep all candidate,
  per-vertex, sort, scan, and CUB workspaces persistent with amortized growth.
- Represent decay with a non-negative inactive-update count. Reset it to zero
  when active and derive the last retained count from
  `pow(decay_factor, count) >= 0.01`.
- Rely on stable CUB radix sorting and old-before-new input order to preserve
  an existing pair's multiplier state. Do not perform cross-thread writes to a
  neighboring sort index.
- Keep the FEM/ABD mass-scaled `per_vertex` penalty as the default. Retain
  conditioning-aware `mu = C * max_i(abs(H_E(i,i)))`, where AL contact is
  disabled while assembling `H_E` and `C` defaults to 0.1, as explicit
  experimental `diag_norm` mode and fall back to `per_vertex` if its estimate
  is invalid.
- Let the AL inner energy line search start at one without IPC's CFL clamp. A
  backtracked step remains inside the same fixed-multiplier solve; multiplier,
  CCD, and collision-free-state updates occur only at a full step.
- Apply `newton/semi_implicit/K_min` to AL's cumulative safe-path weight. Check
  every permitted line-search trial and restore the recorded start point if
  all trials fail rather than accepting an energy-increasing state.
- Use the EE tangent Jacobian for EE friction. Standard IPC retains its
  positive `1e-3` parallel-EE test and complete mollified normal derivatives.
  Its friction path skips detected parallel pairs. AL-IPC uses the shared
  negative disabled threshold in both normal and frictional contact, so every
  AL pair follows its ordinary EE path.
- Use CCD margin 0.001 only for AL-IPC. Preserve 0.1 for IPC.

## Consequences

AL-IPC gains the fork's active-set compactness and optional conditioning
strategy without importing obsolete runtime abstractions or changing IPC
behavior. `mu_scale_fem` and `mu_scale_abd` affect the stable default mode;
`mu_scale_diag_norm` affects only explicitly selected experiments.

The extra per-vertex TOI pass and one non-contact Hessian assembly per frame
have measurable costs. They are accepted only with simulation regressions and
should be compared on contact-rich scenes, where smaller active sets and better
PCG conditioning can amortize them.

This does not claim feature parity with the paper's simulator. Multi-state
`K_min > 1` is implemented, but penalty-free moving boundaries and the
specialized conflict-free analytic PSD Hessian assembly remain separate future
work.

## Alternatives considered

- Cherry-pick all six fork commits: rejected because it imports large assets,
  examples and formatting churn, and overwrites newer solver/runtime work.
- Copy the fork's atomic and fixed threshold literally: rejected for memory
  corruption and decay-rule violations.
- Change every CCD path to margin 0.001: rejected because this task concerns
  AL-IPC and must not silently retune normal IPC.
- Replace `NewtonToleranceManager` with one raw displacement check: rejected
  because it loses subsystem-aware and relative-tolerance behavior.

## Validation

The focused CUDA tests cover exact decay boundaries, earliest-candidate
selection, negative-TOI clamping, preservation of a subnormal double by the
device atomic minimum, `K_min` progress, finite line-search acceptance,
finite-difference gradients for PT, EE, and half-plane friction, and the
parallel-EE policy: a positive threshold detects the standard IPC case while
AL's negative threshold remains false. Configuration tests cover both modes
and reject unknown values. Sample 88 supplies the mixed
cloth/FEM trajectory regression; details and measured failure counts are in
[`2026-09-03-al-ipc-case88-correction.md`](../performance/2026-09-03-al-ipc-case88-correction.md).
The aggregate CUDA/core tests and repository contracts remain required before
merge.
