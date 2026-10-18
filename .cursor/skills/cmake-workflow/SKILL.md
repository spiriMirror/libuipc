---
name: cmake-workflow
description: Build and test the libuipc project using CMake. Use when building the project, running cmake configure/build commands, compiling with RelWithDebInfo, running Catch2 tests, or when the user asks how to build or test libuipc.
---

# CMake Workflow for libuipc

## Build

Run from the repository root and keep generated files in an out-of-source
directory. Reuse the selected toolchain, interpreter and existing build settings.
For a Release configuration with the checked-in Ninja preset:

```bash
cmake -S . -B build --preset ci-release
cmake --build build --config Release -j8
```

Use `RelWithDebInfo` when debugging requires it; for single-config generators,
select it at configure time with `-DCMAKE_BUILD_TYPE=RelWithDebInfo`, and use the
matching build/test configuration. Performance comparisons use `Release`.
Choose parallelism to fit available memory; NVCC can exhaust memory at high `-j`.

See [the maintained build guide](../../../agent_docs/07-build-test-workflow.md)
for Python packaging, dependency setup and CMake/XMake parity.

## Testing

Tests use **Catch2** and are also registered with CTest. The fast CPU gate is:

```bash
ctest --test-dir build --build-config Release --label-regex fast --output-on-failure --timeout 900
```

Run a test executable directly for a focused Catch2 filter.

Find the compiled test binary in the build output (under `<build>/<configuration>/bin/`, for example `build/Release/bin/`) and execute it:

```bash
./path/to/test_executable
```

Catch2 test binaries accept standard flags such as:
- `--list-tests` — list all test cases
- `[tag]` — filter by tag
- `"test name"` — run a specific test by name

## Quick Reference

| Step | Command |
|------|---------|
| Configure | `cmake -S . -B build --preset ci-release` |
| Build | `cmake --build build --config Release -j8` |
| Run tests | Execute the test binary directly |
