# Run Tests

## Overview

Run C++ tests built with Catch2. Test sources are in `apps/tests/`, grouped by feature area.

## Prerequisites

Build the project first using the [build command](./build.md). Test binaries are output to `<cmake-binary-directory>/<configuration>/bin/`.

## Run All Tests in a Binary

```powershell
cd <cmake-binary-directory>/<configuration>/bin
./uipc_test_<name>.exe
```

Test binary names follow the pattern `uipc_test_<name>` where `<name>` matches the CMake target name in `apps/tests/`.

## Run a Specific Test Case

```powershell
./uipc_test_<name>.exe "<test-case-name>"
```

## Run Tests Matching a Tag

Catch2 tags are used to group tests (e.g., `[geometry]`, `[core]`):

```powershell
./uipc_test_<name>.exe [geometry]
```

## Adjust Log Level

```powershell
./uipc_test_<name>.exe --log-level <level>
```

Levels: `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`.

## List Available Tests

```powershell
./uipc_test_<name>.exe --list-tests
```

## Test Organization

| Directory | Area |
|---|---|
| `apps/tests/geometry/` | Geometry algorithms |
| `apps/tests/core/` | Core engine |
| `apps/tests/common/` | Common utilities |
| `apps/tests/backends/` | Backend tests |
| `apps/tests/sim_case/` | Simulation cases |
| `apps/tests/sanity_check/` | Sanity checks |
