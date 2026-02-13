# Format

## Overview

Format C++ code using the `.clang-format` file in the project root.

## Format Changed Files

Format only the files changed relative to `main`:

```bash
git diff --name-only --diff-filter=ACMR main -- '*.h' '*.hpp' '*.cpp' '*.inl' '*.cu' '*.cuh' | xargs clang-format -i
```

## Format a Specific File

```bash
clang-format -i <file>
```

## Format All C++ Files in a Directory

```bash
find <directory> -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.inl' -o -name '*.cu' -o -name '*.cuh' \) -exec clang-format -i {} +
```
