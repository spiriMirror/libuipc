---
name: run-tests
description: Run C++ and Python tests
disable-model-invocation: true
---

# Run Tests

## Overview

Run C++ tests built with Catch2. Test sources are in `apps/tests/`, grouped by feature area.

## Prerequisites

Build the project first using the [build command](../cmake-workflow/SKILL.md). Test binaries are output to `<cmake-binary-directory>/<configuration>/bin/`.

## Run All Tests in a Binary

```bash
cd <cmake-binary-directory>/<configuration>/bin
./uipc_test_<name>
```

Test binary names follow the pattern `uipc_test_<name>` where `<name>` matches the CMake target name in `apps/tests/`. On Windows, the executable will have an `.exe` extension.

## Run a Specific Test Case

```bash
./uipc_test_<name> "<test-case-name>"
```

## Run Tests Matching a Tag

Catch2 tags are used to group tests (e.g., `[geometry]`, `[core]`):

```bash
./uipc_test_<name> '[geometry]'
```

## Adjust Log Level

```bash
./uipc_test_<name> --log-level <level>
```

Levels: `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`.

## List Available Tests

```bash
./uipc_test_<name> --list-tests
```

Keep the single-process `uipc.sim_case` aggregate for global-state pollution
coverage; `scripts/run_sim_case_isolated.py` supplies fresh-process cases
and stable shards. For OR filters, put comma-separated Catch2 names inside
one argument. See [the test guide](../../../agent_docs/07-build-test-workflow.md).

## Python tests (pytest)

Python tests live under `python/tests/`. Run them with the project venv after installing pyuipc (e.g. `uv pip install -e .` or a CMake build that installs into the venv).

To avoid uv re-syncing (and rebuilding) the project every time, use `--no-sync`:

```bash
uv run --no-sync pytest python/tests
```

Or run pytest from the venv directly:

```bash
.venv/bin/pytest python/tests
```

Without `--no-sync`, `uv run` syncs the environment first, which (re)builds and installs pyuipc from source.

The portable default excludes `cuda` and `example`. Select non-interactive
GPU tests with `pytest -m "cuda and not example" python/tests`; interactive
examples require an explicit `-m example` in a GUI environment.

## Test Organization

| Directory | Area |
|---|---|
| `apps/tests/geometry/` | Geometry algorithms |
| `apps/tests/core/` | Core engine |
| `apps/tests/common/` | Common utilities |
| `apps/tests/backends/` | Backend tests |
| `apps/tests/sim_case/` | Simulation cases |
| `apps/tests/sanity_check/` | Sanity checks |
