# Deterministic Mode (Historical Proposal)

> **Status: withdrawn from the active roadmap, 2026-09-11.** The notes below are
> historical, not an agreed implementation plan. The owner requires preserving
> parallel efficiency: small differences from valid floating-point atomic arrival
> orders are expected and do not establish a correctness defect. Do not add sorting,
> fixed-order reductions or serialization solely to eliminate them. A correctness
> change needs evidence of an algorithmic error or a real concurrency/synchronization
> violation. The experimental fixed-order changes were reverted.
>
> There is **no supported `debug/deterministic` configuration key**. The JSON and
> strategies below were proposed, not implemented; the strict scene schema rejects
> the unregistered key. Reconsidering any optional mode requires a new explicit
> requirement, measured costs and separate approval.

## Motivation

The CUDA backend is intentionally parallel-first: floating-point results may
vary slightly between identical runs. That is fine for production simulation,
but it hurts three workflows:

1. **Regression testing** — numerical baselines (e.g. the `sim_case` assertions)
   need tolerances wider than the solver noise, weakening them.
2. **Debugging** — bisecting "assemble → solve → update" for a NaN/divergence is
   much easier when every run is bit-identical (see the deterministic debug
   workflow in the simulation-dev guidelines).
3. **Differentiable simulation** — gradient checks by finite differences require
   a reproducible forward pass.

## Sources of Non-Determinism

| Stage | Source | Location |
| --- | --- | --- |
| System assembly | `atomicAdd` into global gradient/Hessian buffers from unordered threads | `finite_element/`, `affine_body/`, `contact_system/` reporters |
| Linear solve | parallel dot products / norms with tree-order depending on scheduling | `linear_system/` (PCG) |
| Collision detection | BVH build & traversal order, parallel sort with unstable ordering | `collision_detection/` |
| Contact activation | order-dependent insertion into candidate sets | `active_set_system/`, `contact_system/` |

## Proposed Configuration

```json
{
    "debug": {
        "deterministic": false
    }
}
```

- Default `false` — zero overhead in production.
- When `true`, the backend selects deterministic code paths (below).
- The flag is read once in `do_init` and broadcast to all `SimSystem`s via the
  engine config (`ISimSystem::BaseInfo::config`).

## Implementation Strategy (per stage)

1. **Assembly**: replace unordered `atomicAdd` scatter with a two-pass
   "count → sort by target dof → segmented reduce" scheme. Cost: one extra pass
   over the triplets; memory: one index array. This is the classical
   deterministic-assembly tradeoff.
2. **Reductions (dot/norm)**: use fixed-order tree reductions (e.g. the
   warp→block two-level reduction used in the solver's dot kernels). Never
   use atomics for scalar accumulation.
3. **Sorting**: use stable sorts (or sort by (key, index) pairs) in BVH and
   candidate-list construction.
4. **Contact set**: after filtering, sort candidate pairs by
   `(type, id0, id1)` before force evaluation.

Deterministic mode is allowed to be slower (target: < 2x slowdown). It must be
bit-identical on the **same GPU + same driver**; cross-GPU bit-identity is not
required.

## Validation Plan

- Run any `sim_case` twice with `debug/deterministic=true` and assert
  bit-identical retrieved geometries (`SceneIO` snapshot compare).
- Add a dedicated regression test under `apps/tests/regression/` once
  implemented.
- Keep the assertion tolerances of existing `sim_case` tests unchanged;
  deterministic mode may later allow tightening them.

## References

- Simulation debug workflow: `.cursor/skills/simulation-dev/SKILL.md`
- Timer/pipeline stages to instrument first: see the pipeline tree in
  `agent_docs/05-cuda-backend.md`
