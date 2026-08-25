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
2. **Commit in stages, push without asking.** During multi-step work,
   commit each completed stage separately (clear rollback points) and push
   directly — no per-commit confirmation. (Set 2026-08-23.)
3. **Minimal diffs**: make the smallest change that achieves the goal;
   reuse the project's existing algorithm/utility code instead of
   introducing parallel implementations; no opportunistic refactors,
   renames, or cleanups outside the task's scope.
4. **Keep agent_docs current in the same change.** Any update that changes
   structure, workflow, conventions, build, or behavior must update the
   matching doc (and `handoff.md` / `09-known-issues-and-roadmap.md` when
   it closes or opens work) **in the same commit**.

## Language

4. **English only in repo artifacts**: documentation, code comments, commit
   messages. (The conversation language stays whatever the owner uses.)

## Documentation scope

5. **Demo showcases belong on the project homepage, not in the technical
   documentation.** Keep video/demo catalogues in the root README or project
   website; keep `docs/` focused on tutorials, reference, specifications,
   build guidance, and development documentation. The documentation must
   expose a global link back to the project repository. (Set 2026-08-24.)
   Scene-configuration reference material must state every registered key's
   default, unit, effective-value rule, and valid domain/selector where one is
   defined. Representative simulation tutorials must cover rigid bodies, FEM,
   cloth, rigid-soft coupling, and contact in both C++ and Python, verified
   against `libuipc-samples`, public APIs/bindings, and the consuming backend
   implementation. Educational examples are documentation; showcase/demo
   catalogues remain homepage-only. (Expanded 2026-08-24.)

## Build system

6. **clang-format-18 before every C++ commit/PR**: run
   `python scripts/format_changed.py` (formats the C++ files changed vs the
   base ref, CI's exact selection) or `--check` for a dry run. CI rejects
   non-conforming diffs; the pinned local formatter is
   `output/venv_clangfmt/Scripts/clang-format.exe` (18.1.8, same major as
   CI's clang-format-18).
7. **CMake and XMake stay in sync**: every build-affecting change (sources,
   flags, dependencies, pins) must be applied to both build systems, even
   if only one can be verified locally.
8. **No ccache.** It was integrated once and the owner had it reverted; do
   not re-introduce it (or similar compiler-cache layers) without an
   explicit request.

## CUDA backend

9. **Raw kernels only**: business GPU code is written as named
   `__global__` functions with `<<<>>>` launches — no lambda kernels, no
   ParallelFor-style wrappers.
10. **`cuda_tool` is the only device utility layer and must not contain
   muda** (no vendored copy, no submodule, no compat header). Keep
   cuda_tool minimal: no primitive without an in-tree user.
11. **Eigen stays** as the host/device small-matrix dependency. (A
   replacement experiment — geigen under `src/math/` — was tried and
   deliberately reverted; do not resurrect it without an explicit request.)

12. **Use CUB, not Thrust, for new or modified CUDA bulk algorithms.** CUB
   temporary storage must be persistent and reused (normally through
   `cuda_tool`'s per-stream workspace); never allocate/free CUB scratch on
   every call. (Set 2026-08-25.)

## Task-scoped (recorded for context, not general policy)

- During the Stiff-GIPC performance-alignment work: "when a design choice
  is uncertain, follow Stiff-GIPC's algorithm design" — that applied to
  that effort (and its evidence lives in `handoff.md`), not to unrelated
  features.
- During CI watch duty: "you confirm all operations yourself" — scoped to
  that autonomous monitoring task; normal confirmation rules apply
  elsewhere.
