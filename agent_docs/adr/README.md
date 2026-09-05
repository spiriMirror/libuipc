# Architecture Decision Records

ADRs preserve durable design decisions that would otherwise be buried in the
chronological `handoff.md`. Read the relevant ADR before changing a boundary it
owns; source code remains authoritative when implementation and prose differ.

## Index

| ADR | Status | Decision |
|---|---|---|
| [0001](0001-backend-module-compatibility.md) | Accepted | Validate a versioned backend-module handshake before entering the C++ ABI |
| [0002](0002-cuda-component-boundaries.md) | Accepted | Keep one CUDA runtime DLL/device-link with explicit internal source ownership |
| [0003](0003-scene-config-contract.md) | Accepted | Derive typed scene defaults and public schema from one contract |
| [0004](0004-deterministic-sim-system-topology.md) | Accepted | Make SimSystem creation/traversal deterministic and reject strong cycles |
| [0005](0005-test-and-benchmark-entrypoints.md) | Accepted | Separate aggregate pollution tests, isolated shards, and performance evidence |
| [0006](0006-al-ipc-active-set-integration.md) | Accepted | Reimplement AL-IPC active-set and conditioning improvements safely on current CUDA interfaces |
| [0007](0007-embedded-cpp-metis.md) | Accepted | Build the private C++ METIS port inside geometry and remove the external C dependency tree |
| [0008](0008-native-tetrahedralization.md) | Accepted | Native conservative tetrahedralization preserves exact input boundary before quality optimization |

## Lifecycle

- `Proposed`: under discussion; implementation must not depend on it yet.
- `Accepted`: current decision. Amend only to clarify; use a new ADR to reverse
  it.
- `Superseded`: retained for history and linked to its replacement.
- `Rejected`: considered but deliberately not adopted.

Copy [0000-template.md](0000-template.md) for a new record. Use the next number,
one decision per file, English prose, concrete source/test links, and the commit
that first implemented the decision. Do not rewrite old ADRs to make history
look cleaner.
