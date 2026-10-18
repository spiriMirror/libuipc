---
name: simulation-dev
description: General simulation development best practices for correctness, stability, and debuggability. Use when implementing or modifying simulation systems, solvers, constraints, or GPU kernels, especially for index safety, NaN/Inf issues, and diagnostics.
---

# Simulation Development Best Practices

General rules for building simulation code that is correct first, debuggable second, and fast third.

Follow the [owner working agreements](../../../agent_docs/rule.md), especially CUDA rules 9-16.
MUDA is removed; do not restore its dependency, wrappers, or compatibility layer.

## When to Use

Use this skill when:
- adding or modifying simulation systems, solvers, integrators, constraints, or contact logic,
- touching CPU/GPU compute paths that can fail from invalid indices or unstable numerics,
- debugging crashes, divergence, NaN/Inf, or non-deterministic behavior,
- improving diagnostics, assertions, and reproducibility.

## Core Principles

1. Correctness before performance.
2. Fail fast on invalid states.
3. Reproduce with fixed inputs while preserving parallel execution.
4. Use named raw-CUDA kernels and the in-tree `cuda_tool` diagnostics.
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

- Use `UIPC_KERNEL_ASSERT` and the `UIPC_KERNEL_*` diagnostics from
  `src/backends/cuda/cuda_tool/debug.h`; kernel assertions follow `uipc::RUNTIME_CHECK`.
- Use named `__global__` functions and raw `<<<grid, block, 0, stream>>>`
  launches so logs and profiler symbols identify the owning subsystem.
- Use `cuda_tool::BufferView` / `CBufferView` and sparse-matrix views for GPU
  buffers; pass explicit counts/offsets and report their values on failure.
- Keep optional dumps for critical vectors, matrices, topology maps and frame/
  iteration context. Do not use removed `.name()` or `file_line()` wrappers.
- Use scoped timers around hot phases, outside CUDA-graph capture. Gate debug
  dumps and synchronization so the normal path retains its existing overhead.

## CUDA Debug Checklist

For each touched GPU code path:

- Keep index guards before memory access and validate finite intermediates.
- Use `cuda_tool::debug_sync_all()` at diagnostic boundaries to attribute errors;
  remove or gate these barriers after diagnosis.
- Reuse persistent CUB workspace and preserve buffer initialization contracts
  (`resize`, `resize_discard`, `resize_preserve`).
- Keep kernel launch sizing through the existing occupancy helpers and their
  per-kernel-address cache.
- Preserve stream dependencies and fill every reported gradient/Hessian slot.

## Reproduction Workflow

1. Reproduce on the smallest failing case with fixed inputs and random seeds.
2. Compare isolated runs with the single-process aggregate to detect global-state
   pollution; preserve existing concurrent assembly and solver paths.
3. Enable strict checks and locate the first invalid value or missing dependency.
4. Bisect by pipeline stage (assemble -> solve -> update).
5. Distinguish an algorithmic error or synchronization defect from rounding due
   solely to valid atomic-add arrival order. Do not add sorting, fixed-order
   reductions or serialization merely to obtain bitwise-identical trajectories.

## Simulation Debug Loop (Iterative)

Use this loop as the default process for debugging simulation issues:

1. Build a minimal reproducible scene/case that is easy to debug.
2. Use diagnostics to understand behavior, form a concrete assumption, then add debug code to validate that assumption.
3. After validating the assumption, implement the fix in the minimal case first.
4. Re-run the original scene/case to verify the real issue is resolved.
5. Add a regression test for the confirmed fix.

If any step fails or new evidence contradicts the assumption, return to step 1 or 2 and iterate until root cause and fix are both confirmed.

During active GPU debugging, insert `cuda_tool::debug_sync_all()` at key boundaries to stop immediately when a kernel fails, then remove or gate it once diagnosis is complete.

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
