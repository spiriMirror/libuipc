# Rules — Owner-Mandated Working Agreements

Rules the project owner (the user) has explicitly laid down. They override
habit and generic best practice. When one applies to your task, follow it;
when a rule is marked task-scoped, do not generalize it.

Standing rule for this file: **whenever the owner sets or changes a rule,
record it here in the same commit.**

## Change discipline

1. **Commit to the current working branch only.** Never open a PR to (or
   push to) the default branch (`main`) unless the owner explicitly says
   so. (Set 2026-08-23; current working branch at the time:
   `refactor-main`.)
2. **Minimal diffs**: make the smallest change that achieves the goal;
   reuse the project's existing algorithm/utility code instead of
   introducing parallel implementations; no opportunistic refactors,
   renames, or cleanups outside the task's scope.
3. **Keep agent_docs current in the same change.** Any update that changes
   structure, workflow, conventions, build, or behavior must update the
   matching doc (and `handoff.md` / `09-known-issues-and-roadmap.md` when
   it closes or opens work) **in the same commit**.

## Language

4. **English only in repo artifacts**: documentation, code comments, commit
   messages. (The conversation language stays whatever the owner uses.)

## Build system

5. **CMake and XMake stay in sync**: every build-affecting change (sources,
   flags, dependencies, pins) must be applied to both build systems, even
   if only one can be verified locally.
6. **No ccache.** It was integrated once and the owner had it reverted; do
   not re-introduce it (or similar compiler-cache layers) without an
   explicit request.

## CUDA backend

7. **Raw kernels only**: business GPU code is written as named
   `__global__` functions with `<<<>>>` launches — no lambda kernels, no
   ParallelFor-style wrappers.
8. **`cuda_tool` is the only device utility layer and must not contain
   muda** (no vendored copy, no submodule, no compat header). Keep
   cuda_tool minimal: no primitive without an in-tree user.
9. **Eigen stays** as the host/device small-matrix dependency. (A
   replacement experiment — geigen under `src/math/` — was tried and
   deliberately reverted; do not resurrect it without an explicit request.)

## Task-scoped (recorded for context, not general policy)

- During the Stiff-GIPC performance-alignment work: "when a design choice
  is uncertain, follow Stiff-GIPC's algorithm design" — that applied to
  that effort (and its evidence lives in `handoff.md`), not to unrelated
  features.
- During CI watch duty: "you confirm all operations yourself" — scoped to
  that autonomous monitoring task; normal confirmation rules apply
  elsewhere.
