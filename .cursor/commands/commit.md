# Commit

## Overview

Create a well-formatted, conventional commit locally. **Do NOT push unless the user explicitly asks to push.**

## Steps

### 1. Format Changed Files

Run [format command](./format.md) on all changed C++ files before staging.

### 2. Stage Changes

```bash
git add <files>
```

Or stage all changes:

```bash
git add -A
```

Review what's staged:

```bash
git diff --cached --stat
```

### 3. Write Commit Message

Use a temp file to avoid shell escaping issues (`output/` is gitignored):

```bash
mkdir -p output/.cursor
```

Write the commit message to `output/.cursor/commit_msg.txt` following the [commit convention](../rules/commit-convention.mdc).

### 4. Commit

```bash
git commit -F output/.cursor/commit_msg.txt
rm output/.cursor/commit_msg.txt
```

### 5. Push (only if user explicitly requests)

Only push when the user says "push", "commit and push", or similar. Otherwise, stop after step 4.

```bash
git push
```

If the branch has no upstream yet:

```bash
git push -u origin HEAD
```
