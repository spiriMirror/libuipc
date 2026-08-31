# ADR 0005 — Separate Aggregate, Isolated, and Performance Entry Points

- Status: Accepted
- Date: 2026-08-30
- Owners: Test/performance infrastructure
- Implements: `5dec0c29`
- Supersedes: N/A

## Context

The 95 simulation cases share one Catch2 executable. Running them together is
valuable because it exposes leaked process-global CUDA/allocator/cache state,
but it is slow and difficult to shard. Running every case in a new process is
easy to parallelize but cannot replace that pollution signal. Large performance
scenes also lived only in samples/history, without a root-owned invocation or
environment contract.

## Decision

Keep `uipc.sim_case` as the authoritative single-process aggregate. The isolated
runner discovers names from that same binary, sorts them, emits JSON manifests,
and assigns stable round-robin shards; each selected case runs in a fresh
process. CTest aggregate GPU executables share the `uipc_gpu` resource lock.

Keep performance out of portable correctness assertions. Root
`benchmarks/manifest.json` owns stable benchmark names, canonical arguments,
environment cleanup, assets, and metadata paths while implementations remain in
the tracked samples submodule. `run_benchmark.py` preserves child status and
records revisions, runtime/hardware facts, and scene-reported frame timing.

## Consequences

- CI can parallelize deterministic shards without duplicating simulation code.
- Final validation must still run the aggregate at least once.
- GPU CTest processes serialize, while unrelated CPU tests may run concurrently.
- Benchmark variants created with environment overrides are distinct workloads;
  they cannot share a canonical baseline silently.
- Wall-time thresholds require compatible baseline artifacts, not universal CI
  assertions.

## Alternatives considered

- Catch discovery as the default CTest layout: convenient sharding but either
  duplicates or removes the aggregate pollution test.
- Copy case2 into the core repository: creates asset/scene drift from samples.
- Treat benchmark wall time as a regression assertion: unstable across hardware,
  drivers, clocks, and contact iteration variation.

## Validation

Real discovery found 95 cases and produced a 24-case shard 0 of 4; isolated and
aggregate executions passed. CTest exposes the resource lock on all four GPU
aggregates. Script contracts passed 25/25, documentation built, and the promoted
case2 runner completed a one-frame end-to-end run with archived metadata.
