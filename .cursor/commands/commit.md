# Commit

## Overview

Create a well-formatted, conventional commit locally. **Do NOT push unless the user explicitly asks to push.**

## Steps

### 1. Format Changed Files

Run [format command](./format.md) on all changed C++ files before staging.

### 2. Stage Changes

```powershell
git add <files>
```

Or stage all changes:

```powershell
git add -A
```

Review what's staged:

```powershell
git diff --cached --stat
```

### 3. Write Commit Message

Use a temp file to avoid shell escaping issues (`output/` is gitignored):

```powershell
New-Item -ItemType Directory -Force -Path output/.cursor | Out-Null
```

Write the commit message to `output/.cursor/commit_msg.txt` following the [commit convention](../rules/commit-convention.mdc).

### 4. Commit

```powershell
git commit -F output/.cursor/commit_msg.txt
Remove-Item output/.cursor/commit_msg.txt
```

### 5. Push (only if user explicitly requests)

Only push when the user says "push", "commit and push", or similar. Otherwise, stop after step 4.

```powershell
git push
```

If the branch has no upstream yet:

```powershell
git push -u origin HEAD
```
