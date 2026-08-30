# ADR 0001 — Validate Backend Modules Before Entering the C++ ABI

- Status: Accepted
- Date: 2026-08-30
- Owners: Core/backend module boundary
- Implements: `8e0dc35f`
- Supersedes: N/A

## Context

Core runtime-loads `uipc_backend_<name>` and then exchanges C++ virtual objects
and a polymorphic-memory-resource pointer with that DLL. A stale backend can
still export create/destroy symbols while having an incompatible class layout,
libuipc version, or identity. Discovering that mismatch at an arbitrary virtual
call is late and unsafe. CMake also used to change the backend artifact kind when
tests were enabled, so tests and packages did not exercise the same boundary.

## Decision

Every backend is a runtime-loadable shared library in every configuration. It
must export `uipc_query_module` in addition to init/create/destroy. Before PMR
synchronization or engine construction, Core validates the size-versioned
`UIPCBackendModuleInfo`, exact backend ABI version, libuipc major/minor version,
and requested backend identity. Any mismatch fails immediately with a module
diagnostic.

Patch versions may differ; major/minor or backend ABI changes require a matching
module. The handshake protects entry to the existing C++ interface—it does not
turn the entire backend interface into a stable C ABI.

## Consequences

- Mixed installs fail at load time instead of risking a virtual-call crash.
- CMake/XMake tests, local builds, and wheels use the same artifact semantics.
- A backend built before this contract is intentionally rejected as an old or
  non-libuipc module.
- Future ABI fields must extend the size-versioned record compatibly or bump the
  backend ABI.

## Alternatives considered

- Rely on create/destroy symbol presence: cannot detect compatible names with an
  incompatible C++ layout.
- Encode compatibility only in filenames: fragile under copied/renamed DLLs.
- Replace the full interface with C immediately: a larger migration not needed
  to make the present boundary fail safely.

## Validation

Release none/CUDA DLLs exported all four symbols. Core passed 36 cases / 988
assertions through the loader path, and CUDA backend tests passed 12 cases / 238
assertions.
