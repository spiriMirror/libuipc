---
name: format
description: Format C++ code using clang-format
disable-model-invocation: true
---

# Format

## Overview

Format C++ code using the `.clang-format` file in the project root.

## Format Changed Files

Use the repository helper with clang-format 18 (the same major as CI):

```bash
python scripts/format_changed.py
python scripts/format_changed.py --check
```

The helper selects committed C++ changes from the merge base of `origin/main`
to `HEAD`; use `--base <ref>` when a different base is intended. Format newly
edited, uncommitted files explicitly before staging them. Formatter lookup and
the pinned Windows executable are documented in
[the build guide](../../../agent_docs/07-build-test-workflow.md).

## Format a Specific File

```bash
clang-format-18 -i <file>
```

## Format All C++ Files in a Directory

```bash
find <directory> -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.inl' -o -name '*.cu' -o -name '*.cuh' \) -exec clang-format-18 -i {} +
```
