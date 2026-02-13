# Format

## Overview

Format C++ code using the `.clang-format` file in the project root.

## Format Changed Files

Format only the files changed relative to `main`:

```powershell
git diff --name-only --diff-filter=ACMR main -- '*.h' '*.hpp' '*.cpp' '*.inl' '*.cu' '*.cuh' | ForEach-Object { clang-format -i $_ }
```

## Format a Specific File

```powershell
clang-format -i <file>
```

## Format All C++ Files in a Directory

```powershell
Get-ChildItem -Recurse -Include *.h,*.hpp,*.cpp,*.inl,*.cu,*.cuh <directory> | ForEach-Object { clang-format -i $_.FullName }
```
