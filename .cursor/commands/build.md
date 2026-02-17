# Build

## CMake Build Process

Always **configure then build** from the cmake binary directory. Skipping configure may cause link errors from missing source files.

**NOTE**: Developers may not always use the `build/` directory as the cmake binary directory. It may be `<project-root>/../CMakeBuild/`, or follow the CLion convention `<project-root>/cmake-build-<configuration>`. Check the project structure and find the correct build directory before running commands.

```bash
cd <cmake-binary-directory>
cmake -S <project-root> -DCMAKE_BUILD_TYPE=<TYPE> [OPTIONS]
cmake --build . --config <TYPE> -j<N>
```

- `TYPE`: `RelWithDebInfo` (default for dev), `Release` (production), or `Debug` (full debug, slow).
- `OPTIONS`: See [CMakeLists.txt](../../CMakeLists.txt) for available options.
- `N`: Number of parallel jobs (use the number of CPU cores).

## Build Types

| Type | Use |
|---|---|
| `RelWithDebInfo` | Default for development (fast + debug info) |
| `Release` | Production / benchmarks |
| `Debug` | Full debug (slow) |

## Build a Specific Target

```bash
cmake --build . --target <TARGET_NAME> --config <TYPE> -j<N>
```

## Reference

See [docs/build_install/](../../docs/build_install/) for full build & install guides.
