---
name: review-pr
description: >-
  Review a libuipc pull request end-to-end: checkout the PR, summarize changes,
  list files for the human reviewer, and perform a domain-aware AI review
  covering physics correctness, backend architecture, C++ style, GPU code,
  and Python bindings. Optionally post review comments via `gh`. Use when the
  user provides a PR number, asks to review a PR, or wants to inspect a libuipc
  GitHub PR.
---

# libuipc PR Review

## Architecture primer (read before reviewing)

libuipc is a GPU-accelerated Incremental Potential Contact (IPC) simulation
library (C++20 + optional Python/pybind11).

**Layer split:**

| Layer | Path | Role |
|-------|------|------|
| Core | `src/core/`, `include/uipc/` | Engine lifecycle, Scene/World, geometry attribute ECS |
| Geometry | `src/geometry/` | Mesh algorithms; libigl, TBB, Octree, METIS |
| Constitution | `src/constitution/` | Material / constraint implementations |
| I/O | `src/io/` | Scene + mesh serialisation |
| Sanity check | `src/sanity_check/` | Pre-sim scene validation |
| Backends | `src/backends/<name>/` | Physics engines as runtime-loaded **MODULE** DLLs |
| Pybind | `src/pybind/` | Thin pybind11 wrapper; mirrors C++ submodules |

**Backend plugin ABI** (`src/backends/common/module.h`):
- Exports: `uipc_init_module`, `uipc_create_engine`, `uipc_destroy_engine`
- Base classes: `SimEngine` → `ISimSystem` → subsystem folders (`affine_body/`,
  `finite_element/`, `contact_system/`, `collision_detection/`, etc.)
- RMR (Reporter–Manager–Receiver) data-oriented design; avoid heavy OOP in backend code

**CUDA backend subsystems** (most review-sensitive):
`affine_body/`, `finite_element/`, `contact_system/`, `collision_detection/`,
`distance_system/`, `active_set_system/`, `inter_primitive_effect_system/`,
`engine/` (Newton + augmented-Lagrangian step in `advance_al.cu`)

## Workflow

### 1. Checkout and fetch metadata

```bash
gh pr checkout <PR_NUMBER>
gh pr view <PR_NUMBER> --json title,body,author,baseRefName,headRefName,headRefOid,repository,url,additions,deletions,changedFiles
gh pr diff <PR_NUMBER> --name-only
```

### 2. Present summary to the human reviewer

```
## PR #<N>: <title>
Author: <author>   Branch: `<head>` → `<base>`
URL: <url>
Stats: +<add> / -<del> across <N> files

### Description
<body>

### Files to review  (grouped by subsystem)
#### Backend — CUDA physics
  src/backends/cuda/affine_body/...
  src/backends/cuda/finite_element/...
  ...
#### Core / Public API
  include/uipc/...
  src/core/...
#### Geometry / Constitution
  src/geometry/...   src/constitution/...
#### Python bindings
  src/pybind/...   python/src/uipc/...
#### Build / CMake
  CMakeLists.txt   src/backends/.../CMakeLists.txt
#### Tests
  apps/tests/...   python/tests/...
```

### 3. AI review — libuipc-specific checklist

Run `gh pr diff <PR_NUMBER>` and check every area below that is touched.

---

#### Fast fail
- [ ] All expected internal invariants are checked with `UIPC_ASSERT` where feasible
- [ ] All user-facing inputs are validated with `UIPC_ASSERT_THROW`

#### C++20 style
- [ ] C-style code is forbidden unless unavoidable (e.g., DLL export signatures)
- [ ] Raw pointer parameters are forbidden unless unavoidable; prefer `span<T>`
- [ ] `const std::string&` parameters are forbidden; use `std::string_view` instead
- [ ] Multiple inheritance is forbidden; separate concerns into distinct classes

#### CUDA / GPU code
- [ ] GPU buffer arguments use `muda::BufferView` / `muda::TripletMatrixView` (or equivalent view types) instead of raw pointers
- [ ] Raw pointer parameters are forbidden in device code unless unavoidable

#### Constitution / material
- [ ] New constitutions implement required `IConstitution`-family interface
- [ ] Material parameters validated / clamped (no negative stiffness, density, etc.)

#### Public C++ API (`include/uipc/`)
- [ ] No breaking changes to public headers without a version bump or deprecation notice
- [ ] New public classes/functions follow existing naming: `CamelCase` types,
  `snake_case` functions, `m_` prefix for members
- [ ] `#pragma once` present; no `<windows.h>`-style global macros leaked

#### C++ style (`.clang-format` enforced)
- [ ] 4-space indent, 80-column limit
- [ ] Allman braces (`{` on its own line)
- [ ] Left-aligned pointers (`int* p`, not `int *p`)
- [ ] No `using namespace std;` in headers

#### CMake
- [ ] New backend follows `uipc_add_backend(name)` macro in `src/backends/CMakeLists.txt`
- [ ] New CMake options added to the root `CMakeLists.txt` option block with sensible defaults
- [ ] `file(GLOB ...)` patterns extended if new source subdirs added

#### Python bindings (`src/pybind/` + `python/`)
- [ ] Check whether new features expose a public API that should be mirrored in the Python bindings
- [ ] New C++ types exposed via matching pybind submodule (mirrors C++ namespace)
- [ ] `__init__.py` import chain not broken (`pyuipc` → `init()` → `config["module_dir"]`)
- [ ] Python-side tests added under `python/tests/` for new pybind surface

#### Tests
- [ ] C++ tests use Catch2 macros; added via `uipc_add_test` in the appropriate
  `apps/tests/<subsystem>/CMakeLists.txt`
- [ ] Regression tests added for bug fixes

---

### 4. Output format

Each finding must have a **sequential number** (1, 2, 3 … across all severity tiers) and a
**clickable local file link** constructed from the absolute repo path + relative file path.
Use this markdown link format for every citation:

```
[filename.ext:LINE](ABSOLUTE_REPO_PATH/relative/path/to/file.ext)
```

For example, if the repo is at `/home/user/libuipc` and the file is
`src/backends/cuda/affine_body/foo.cu` at line 42:

```
[foo.cu:42](/home/user/libuipc/src/backends/cuda/affine_body/foo.cu)
```

Full output template:

```
## AI Review — PR #<N>

### Summary of changes
<one paragraph>

### Findings

🔴 Critical (must fix)
**#1** — [filename.ext:LINE](ABSOLUTE_PATH/relative/path/filename.ext)
Description of the issue and why it matters.

**#2** — [filename.ext:LINE](ABSOLUTE_PATH/relative/path/filename.ext)
Description.

🟡 Suggestion
**#3** — [filename.ext:LINE](ABSOLUTE_PATH/relative/path/filename.ext)
Description.

🟢 Nice-to-have
**#4** — [filename.ext:LINE](ABSOLUTE_PATH/relative/path/filename.ext)
Description.

### Checklist coverage
(tick the items above that were verified or N/A)
```

---

### 5. Post comments via `gh` (when asked)

**General review comment / request changes / approve:**
```bash
gh pr review <PR_NUMBER> --comment   --body "..."
gh pr review <PR_NUMBER> --request-changes --body "..."
gh pr review <PR_NUMBER> --approve   --body "LGTM"
```

**Inline comment on a specific line:**
```bash
gh api repos/<OWNER>/<REPO>/pulls/<PR_NUMBER>/comments \
  --method POST \
  -f path="<FILE_PATH>" \
  -f body="Your comment" \
  -F line=<LINE_NUMBER> \
  -f side="RIGHT" \
  -f commit_id="<HEAD_REF_OID>"
```

Run `gh auth status` first if commands fail.
