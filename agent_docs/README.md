# agent_docs — Project Tour for AI Agents

This directory is a structured summary of the libuipc codebase, allowing newly onboarded AI agents (or new developers) to quickly build an accurate understanding of the project **without reading all the source code**. All content is compiled from actually reading the source code and `docs/`, with key file paths annotated.

> Suggested usage: read this README and `01-project-overview.md` first to build a global picture, then consult the corresponding topic document as your task requires. If a summary conflicts with the source code, **the source code prevails**.

## Document Index

| File | Content |
|---|---|
| **`rule.md`** | **Owner-mandated working agreements — read first, they override habit** (branch/commit policy, minimal diffs, doc sync, English-only artifacts, CMake+XMake parity, raw kernels, no muda/ccache/geigen) |
| `01-project-overview.md` | Project positioning, top-level directories, three core concepts, typical simulation workflow |
| `02-core-architecture.md` | Three-layer architecture, Engine/World/Scene lifecycle, RMR pattern, backend plugin ABI |
| `03-geometry-and-io.md` | SimplicialComplex, attribute system, geometry algorithms, IO classes |
| `04-constitutions.md` | Full list of constitution models (UID, parameters, physical meaning), Constraints and joints |
| `05-cuda-backend.md` | CUDA backend subsystems, advance pipeline, cuda_tool (raw-CUDA utilities) and kernel naming, performance analysis |
| `06-python-api-and-packaging.md` | pybind structure, Python package layout, wheel packaging pipeline |
| `07-build-test-workflow.md` | CMake/XMake builds, test system, CI drift incidents and the pin/overlay pattern, development conventions (summary of .cursor rules) |
| `08-pitfalls-and-debugging.md` | **Collected hard-won pitfalls**: build/CI traps, suite pollution, perf-measurement pitfalls, Python/runtime API traps, contact/constraint semantics gotchas, cuda_tool contracts |
| `09-known-issues-and-roadmap.md` | **Open issues and plans**: remaining Stiff-GIPC perf gap with levers, deferred CFL floor, dependency pins to unwind, external PR review status, samples repo state |
| `10-code-navigation-and-extension-map.md` | Public API → implementation → backend → binding → test map; coordinated change recipes and verification matrix |
| `11-scene-data-lifecycle-and-serialization.md` | Scene ownership, current/rest geometry, pending mutations, Animator, contact/subscene semantics, snapshots and serialization limits |
| `12-secondary-modules-samples-and-docs.md` | Features, sanity checking, DiffSim, USD/VDB, Python helpers, samples index, MkDocs/API generation, deployment and release boundaries |
| `adr/` | Accepted architecture decisions, consequences, alternatives, and validation; consult before changing owned boundaries |
| `performance/` | Durable benchmark evidence, interpretation policy, rejected experiments, and evidence template |
| `handoff.md` | Chronological session history and transient operational detail; durable decisions/evidence route to `adr/` and `performance/` |

## 30-Second Overview

- **libuipc** = a C++20 cross-platform GPU physics simulation library implementing Unified IPC (Incremental Potential Contact), unifying simulation of rigid bodies/soft bodies/cloth/rods with penetration-free frictional contact. Corresponding papers: GIPC 2024, StiffGIPC 2025 (Siggraph).
- The user API has three conceptual layers: **Engine** (algorithm + backend) → **World** (`init/advance/retrieve`) → **Scene** (snapshot: Objects/Geometries/Constitutions/Contacts/Animator).
- Backends are runtime-loaded shared libraries (`uipc_backend_cuda`, `uipc_backend_none`). Before init/create/destroy, `uipc_query_module` validates the backend ABI, libuipc major/minor version, and backend identity; test and packaged builds use the same library form.
- Inside a backend, an ECS-style architecture of **DOP + RMR (Reporter-Manager-Receiver)** is used; all simulation functionality is implemented as `SimSystem` derived classes, auto-registered via the `REGISTER_SIM_SYSTEM` macro.
- All GPU kernels are named `__global__` functions launched via raw `<<<>>>` (device utilities uniformly come from the in-house `src/backends/cuda/cuda_tool/`; no muda dependency; Eigen is retained).
- The constitution layer spans FEM/ABD materials, shells/rods, constraints,
  stitching, and joint families. Some algebra is generated from the
  `scripts/symbol_calculation/` notebooks, while other models use handwritten or
  ported analytic derivatives; header count is not model count (doc 04).
- Dual build systems: CMake (primary) + XMake (backup); dual APIs: C++ + Python (pybind11, PyPI package name `pyuipc`).
- Tests: Catch2 (`apps/tests/`, currently 95 sim-case source files; IDs are not unique/contiguous) + pytest (`python/tests/`, 19 files / 75 top-level tests).

## Onboarding Checklist for New Agents

1. **Read `rule.md` first** — owner-mandated working agreements (branch policy, minimal diffs, doc sync, language, build parity). Then read `10-code-navigation-and-extension-map.md` for the source path, `09-known-issues-and-roadmap.md` for open work, the relevant ADR, and the relevant topic guide. Use `handoff.md` only when historical detail is needed.
2. Before coding, must read `.cursor/rules/cpp-format.mdc` (C++ style) and the conventions summary in `agent_docs/07-build-test-workflow.md`.
3. Before modifying solver/constraints/GPU kernels, read `.cursor/skills/simulation-dev/SKILL.md` (index safety, NaN checks, debugging workflow).
4. For build/test commands see `07-build-test-workflow.md`; for GPU performance optimization read `performance/README.md` before `.cursor/skills/gpu-optimization/SKILL.md`.
5. Commits follow Conventional Commits (summarized in the `07` document).

## Known Documentation Drift Notes (as of 2026-08)

- `docs/` targets users (mkdocs site), `agent_docs/` targets agents; the two complement each other.
- The C++ GUI has been removed (the former `include/uipc_gui/`, `src/gui/`, `apps/tests/gui/` and the `UIPC_BUILD_GUI` option have been deleted); use the Python-side `uipc.gui` (polyscope) for visualization.
- The `none` backend is an empty implementation (for templates/debugging); **simulation cannot run without a GPU environment**.
- Diff-Sim is marked "Coming Soon"; the related APIs may change.
- The UID tables in `docs/specification/constitution_uid.md` and `implicit_geometry_uid.md` are generated by `scripts/gen_uid_doc.py`. The parser and its regression test cover both designated and statement-assigned `UIDInfo` registrations; the docs workflow enforces `--check`.
- The deterministic execution mode currently only has a design document (`docs/development/deterministic_mode.md`) and is not yet implemented.
- `libuipc-samples/` is a tracked submodule with 52 current example directories; numbering is historical and includes two `40_*` directories.
- The incremental Scene commit path has verified topology/ID/subscene limitations; read doc 11 before treating it as a general replication protocol.
- XMake explicitly disables ccache, mirrors the active optional USD/VDB modules, and synchronizes pybind package copies before packaging.
