# ADR 0002 — Keep One CUDA Module with Explicit Internal Components

- Status: Accepted
- Date: 2026-08-30
- Owners: CUDA backend/build systems
- Implements: `d5b75775`, extended by `a140bfd4`
- Supersedes: N/A

## Context

The CUDA backend had nearly two hundred compiled sources behind one target. That
obscured ownership and made architectural changes hard to review or roll back.
Splitting it into runtime DLLs, however, would cross static SimSystem
registration, relocatable device-code calls, PMR ownership, and the C++ backend
ABI, while requiring multiple CUDA device links.

## Decision

Keep one `uipc_backend_cuda` shared library and one final RDC device link.
Partition its source ownership into runtime, affine body, collision,
contact/effects, FEM, linear system, coupling, plus the optional
legacy-collision component. `components.cmake` and `components.lua` are matching
ownership manifests; configuration rejects an unowned or multiply-owned
compiled source. CMake realizes the partition with internal OBJECT targets.
XMake attaches the matching groups directly to the shared target because its
built-in CUDA device-link does not collect object files from OBJECT dependencies.

The V0, stackless, and linear-BVH simplex trajectory filters belong to the
logical `collision_legacy` component (`cuda_collision_legacy_objects` in
CMake). They remain enabled by default for compatibility but can be omitted
with the corresponding CMake/XMake option.

## Consequences

- Domain ownership and build/rollback boundaries are visible without adding a
  runtime ABI boundary.
- All static registrars still land in one module and device symbols resolve once.
- CMake and XMake manifests must change together whenever a source is added,
  removed, or reassigned.
- CMake OBJECT targets are internal implementation details; XMake intentionally
  keeps CUDA sources on the final target so device-link sees every RDC object.
  Users still load one DLL.
- A compatibility build owns 198 sources; a lean legacy-off build compiles 195.
- CMake generates one additional comment-only CUDA-language anchor in the build
  directory. It is not a functional/domain source: it makes Visual Studio
  schedule the final RDC device-link when all real CUDA code comes from OBJECT
  targets. Ninja already linked correctly; XMake directly owns all sources on
  the final target and needs no equivalent anchor.

## Alternatives considered

- Keep one monolithic source glob: simple but provides no ownership invariant.
- One DLL per domain: clearer runtime modules but unacceptable registration,
  device-link, and ABI complexity for the current architecture.
- Move all files into new directories immediately: high-churn renames without
  additional enforcement.

## Validation

Both manifests accept the same inventory and reject missing/duplicate ownership.
Default and legacy-off CMake/XMake configurations built; all four advertised
collision selectors were instantiated in the default build, while the lean
schema/DLL exposed only the default. The full simulation suite passed 95 cases /
14212 assertions. A later CI audit confirmed that XMake OBJECT dependencies did
not enter device-link; the accepted decision was corrected to retain logical
ownership while attaching XMake CUDA sources directly to the shared target.
