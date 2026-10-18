---
name: commit
description: Create a well-formatted conventional commit
disable-model-invocation: true
---

# Commit

## Overview

Create a conventional commit on the current working branch. Follow the
[owner working agreements](../../../agent_docs/rule.md): commit completed stages and push the working
branch under the standing owner authorization. Pushing to `main` or opening
a PR targeting it requires an explicit owner request.

## Steps

### 1. Format Changed Files

Run [format command](../format/SKILL.md) on all changed C++ files before staging.

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

Write the commit message to `output/.cursor/commit_msg.txt` following the [commit convention](../commit-convention/SKILL.md).

### 4. Commit

```bash
git commit -F output/.cursor/commit_msg.txt
rm -r output/.cursor
```

### 5. Push the Working Branch

Use the current branch and its verified upstream under the owner policy.
Do not push to `main` without an explicit owner request.

```bash
git push
```

If the branch has no upstream yet:

```bash
git push -u origin HEAD
```
