# ADR 0003 — Derive Scene Defaults and Schema from One Contract

- Status: Accepted
- Date: 2026-08-30
- Owners: Core scene configuration
- Implements: `ec7253a7`, extended by `a140bfd4`
- Supersedes: N/A

## Context

Scene configuration previously maintained typed defaults and metadata in two
parallel lists. A key could be added, renamed, or retuned in only one list,
leaving runtime behavior, Python/C++ documentation, validation, and agent tooling
in disagreement. Some selector values also depend on compiled backend
capabilities.

## Decision

`make_scene_config_contract()` declares each key once with its typed default,
JSON/storage type, description, consumers, units, constraints, lifecycle, and
status. `Scene::default_config()` and `Scene::config_schema()` derive from that
same contract. Scene construction and `World::init(scene)` validate unknown
keys, storage types, finite values, selectors, bounds, and cross-field rules.

Build-conditional values remain part of the same entry. In particular, the
collision-method enum and `conditionalValues` metadata reflect whether the
legacy CUDA component was actually built.

## Consequences

- A default cannot change independently of the machine-readable schema.
- C++/Python tools can query one strict contract instead of scraping backend
  code.
- Adding a key still requires its consuming implementation, tests, and human
  documentation; the contract prevents metadata drift, not unused knobs.
- Backend-derived effective-value rules (for example relative tolerances and
  kappa corridors) remain documented beside the consumer.

## Alternatives considered

- Generate C++ from a JSON/YAML schema: stronger language independence but adds
  a code-generation/bootstrap step for typed Eigen/unit defaults.
- Keep two lists with a size/name assertion: detects missing names but not
  inconsistent defaults or metadata.
- Let backends accept arbitrary JSON: makes typos and unsupported options silent.

## Validation

The initial refactor produced a byte-for-byte equivalent normalized schema and
passed focused C++ schema tests, Core 36/988, and Python 5/5. The later
build-conditional extension passed both legacy ON/OFF schemas, Core 36/1001,
Python 5/5 in each configuration, and actual selector-to-SimSystem integration.
