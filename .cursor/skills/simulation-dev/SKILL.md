---
name: simulation-dev
description: General simulation development best practices for correctness, stability, and debuggability. Use when implementing or modifying simulation systems, solvers, constraints, or GPU kernels, especially for index safety, NaN/Inf issues, and diagnostics.
---

# Simulation Development Best Practices

General rules for building simulation code that is correct first, debuggable second, and fast third.

## When to Use

Use this skill when:
- adding or modifying simulation systems, solvers, integrators, constraints, or contact logic,
- touching CPU/GPU compute paths that can fail from invalid indices or unstable numerics,
- debugging crashes, divergence, NaN/Inf, or non-deterministic behavior,
- improving diagnostics, assertions, and reproducibility.

## Core Principles

1. Correctness before performance.
2. Fail fast on invalid states.
3. Prefer deterministic behavior during debugging.
4. Prefer MUDA-native diagnostics for GPU paths whenever possible.
5. Validate data at subsystem boundaries.
6. Instrument first, optimize after evidence.

## Safety Rules (Always Apply)

### Input and shape validation

- Validate counts, offsets, dimensions, and index ranges before compute.
- Verify assumptions at boundaries (API input, scene load, assembly, solver entry).
- Use clear sentinel conventions and assert them (`-1`, empty, null, etc.).

### Index safety for GPU/parallel code

- Guard every computed index before read/write.
- Return early on invalid thread work instead of continuing with partial writes.
- Prefer host-side prevalidation to avoid propagating bad mappings into kernels.

### Numerical robustness

- Check for finite values (`isfinite`) at key checkpoints.
- Assert on first invalid scalar near source (residuals, energies, step sizes, norms).
- Add conservative guards for divide-by-near-zero and invalid square roots/logs.

## Assertion Best Practices

Every assertion should answer:
1. What failed?
2. Where in the pipeline?
3. With what values?
4. What is the likely cause?

Checklist for high-quality assert messages:
- include stage/iteration/frame context,
- include offending value and expected range/invariant,
- include at least one hint for root-cause direction,
- keep wording short and actionable.

## Diagnostics and Observability

- Use MUDA diagnostics as the default for GPU debugging:
  - attach file/line metadata on launches,
  - assign explicit names to views/buffers,
  - prefer MUDA launch wrappers over raw CUDA launch sites when practical.
- Attach source location metadata to kernel/parallel launches when available.
- Name critical buffers/views to make reports readable.
- Keep optional debug dumps for important intermediates (vectors, matrices, topology maps).
- Use scoped timers around major phases and hot sub-phases.
- Make debug instrumentation easy to toggle with config flags.

## MUDA Debug Checklist

For each touched GPU code path:

- Do: add launch source mapping (for example, `file_line(__FILE__, __LINE__)`).
- Do: name captured views/buffers (for example, `viewer().name("x")`) for readable diagnostics.
- Do: use MUDA launch/parallel wrappers as first choice for new or modified kernels.
- Do: use `muda::debug_sync_all()` as a fast-fail barrier while debugging broken kernels.
- Do: keep index guards at the top of kernel bodies before any memory access.
- Do: preserve a low-overhead path when debug flags are off.
- Don't: add raw CUDA launch sites unless there is a clear, documented need.
- Don't: leave anonymous captures for critical buffers in hot or unstable paths.
- Don't: rely on post-mortem logs only; add immediate context at the failure point.

## Deterministic Debug Workflow

1. Reproduce on the smallest case that still fails.
2. Freeze nondeterminism where possible:
   - fixed seeds,
   - stable ordering,
   - reduced parallel variability.
3. Enable strict checks and diagnostic output.
4. Bisect by pipeline stage (assemble -> solve -> update).
5. Narrow to first bad value and assert there.

## Simulation Debug Loop (Iterative)

Use this loop as the default process for debugging simulation issues:

1. Build a minimal reproducible scene/case that is easy to debug.
2. Use diagnostics to understand behavior, form a concrete assumption, then add debug code to validate that assumption.
3. After validating the assumption, implement the fix in the minimal case first.
4. Re-run the original scene/case to verify the real issue is resolved.
5. Add a regression test for the confirmed fix.

If any step fails or new evidence contradicts the assumption, return to step 1 or 2 and iterate until root cause and fix are both confirmed.

During active GPU debugging, insert `muda::debug_sync_all()` at key boundaries to stop immediately when a kernel fails, then remove or gate it once diagnosis is complete.

## Convergence and Stability Guidelines

- Separate stopping criteria from failure criteria.
- Distinguish "did not converge yet" from "invalid state".
- Track monotonic indicators where expected (energy, residual envelopes).
- For iterative methods, sample/check periodically to limit synchronization overhead.
- Provide explicit fallback or abort policy for non-convergence in strict mode.

## Change Workflow

1. Write/modify logic with invariants first.
2. Add assertions at boundaries and first-use sites.
3. Add diagnostics (timers, buffer names, optional dumps).
4. Verify on:
   - minimal case,
   - representative case,
   - stress case.
5. Only then profile and optimize.

## Code Review Checklist

- Are assumptions explicit and validated?
- Can invalid index/data reach compute kernels?
- Are numerical hazards guarded and diagnosed?
- Are failure messages actionable?
- Is there enough instrumentation to localize failures quickly?
- Are debug paths configurable and low-overhead when disabled?

## Optional Project Adaptation

Map this checklist to your project conventions:
- assertion macro(s),
- strict/debug config flags,
- timer/profiler hooks,
- dump format and output locations,
- solver-specific triage metrics.
