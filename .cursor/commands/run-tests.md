# Run Tests

## Overview

Run C++ tests built with Catch2. Test sources are in `apps/tests/`, grouped by feature area.

## Prerequisites

Build the project first using the [build command](./build.md). Test binaries are output to `<cmake-binary-directory>/<configuration>/bin/`.

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

## Test Organization

| Directory | Area |
|---|---|
| `apps/tests/geometry/` | Geometry algorithms |
| `apps/tests/core/` | Core engine |
| `apps/tests/common/` | Common utilities |
| `apps/tests/backends/` | Backend tests |
| `apps/tests/sim_case/` | Simulation cases |
| `apps/tests/sanity_check/` | Sanity checks |
