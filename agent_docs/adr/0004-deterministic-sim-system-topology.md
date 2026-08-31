# ADR 0004 — Make SimSystem Topology Deterministic and Acyclic

- Status: Accepted
- Date: 2026-08-30
- Owners: Backend SimSystem infrastructure
- Implements: `2fb20205`
- Supersedes: N/A

## Context

Static registrars arrived in linker/initialization order, exact lookup used raw
`type_info::hash_code()`, and collection traversal used an unordered map. Build,
compatible override selection, action registration, invalidation logs, and
`systems.json` could therefore depend on toolchain layout. Strong dependency
cycles had no direct diagnostic.

## Decision

Registrars retain their complete demangled type name, and `SimEngine` sorts them
ordinally before construction. Exact lookup uses collision-safe
`std::type_index`; a separate ordered vector governs build, compatible lookup,
invalidation, formatting, and JSON. Compatible derived lookup skips systems
already invalidated by selectors or missing dependencies. After invalidity
cascades settle, the active strong-dependency graph must be acyclic; otherwise
initialization throws with the full `A -> B -> A` path.

## Consequences

- The same binary produces stable system manifests and selection order.
- Type renames can change the default order. Systems must express semantics via
  dependencies/dispatchers, never source or registration priority.
- Weak dependency cycles remain permitted because they do not define ownership
  or invalidation requirements.
- Internal exact lookup no longer carries a theoretical hash-collision failure.

## Alternatives considered

- Sort the unordered map only when dumping: fixes diagnostics but not behavior.
- Assign manual integer priorities: creates a second global registry and hides
  real dependencies.
- Topologically build systems: dependencies are discovered by `do_build()`, so a
  separate declaration protocol would be required first.

## Validation

Dependency-graph tests cover acyclic graphs, disabled nodes, multi-node cycles,
and self-cycles. Core/Common tests, CUDA backend 12/238, IPC and AL-IPC manifests,
and the full 95-case / 14212-assertion simulation suite passed.
