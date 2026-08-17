## Summary

<!-- What changed and why. Link related issues with `Fixes #123`. -->

## Breaking Changes

<!-- List any public API / behavior changes, or write "None". -->

## Checklist

### Fast fail
- [ ] Internal invariants checked with `UIPC_ASSERT` where feasible
- [ ] User-facing inputs validated with `UIPC_ASSERT_THROW`

### C++ style
- [ ] Conforms to `.clang-format` (4-space indent, 80 cols, Allman braces, left-aligned pointers `int* p`)
- [ ] No raw pointer parameters (use `span<T>` / `muda::BufferView`); no `const std::string&` parameters (use `std::string_view`); no multiple inheritance
- [ ] Naming: `CamelCase` types, `snake_case` functions, `m_` member prefix; `#pragma once` in headers

### GPU / CUDA (if touched)
- [ ] GPU buffers passed as `muda::BufferView` / `muda::TripletMatrixView` (or equivalent view types)
- [ ] Index guards at the top of kernel bodies; no NaN/Inf hazards introduced

### Constitution / material (if touched)
- [ ] New constitutions implement the required `IConstitution`-family interface and use an assigned UID
- [ ] Material parameters validated / clamped (no negative stiffness, density, etc.)

### Build / bindings (if touched)
- [ ] New CMake options added to the root `CMakeLists.txt` option block; new source subdirs added to `file(GLOB ...)` and `xmake.lua`
- [ ] New public C++ API mirrored in the Python bindings (`src/pybind/`), without breaking the `__init__.py` import chain

### Tests
- [ ] C++ tests added via `uipc_add_test` under `apps/tests/<subsystem>/` and pass locally
- [ ] Python tests added under `python/tests/` for new binding surface
- [ ] Bug fixes include a regression test (existing tests were NOT modified to make the fix pass)
