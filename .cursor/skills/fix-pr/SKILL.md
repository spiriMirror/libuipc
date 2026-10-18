---
name: fix-pr
description: Fix a pull request based on review feedback
disable-model-invocation: true
---

# Fix PR

## Overview

Fix a pull request based on review feedback comments.

## 1. Identify the PR

If no PR number is provided, list and ask the developer to pick:

```bash
gh pr list --repo spiriMirror/libuipc
```

**Note:** Always use `--repo spiriMirror/libuipc` to reference the upstream repository (works with forks).

## 2. Verify the Working Branch

Follow the [owner working agreements](../../../agent_docs/rule.md). Keep
implementation commits on the current working branch; honor any different
working branch explicitly selected by the owner.

## 3. Read Review Comments

```bash
gh pr view <PR_NUMBER> --comments --repo spiriMirror/libuipc
```

If the output is too long, store it in a temporary file for reference (`output/` is gitignored):

```bash
mkdir -p output/.cursor
gh pr view <PR_NUMBER> --comments --repo spiriMirror/libuipc > output/.cursor/pr_comments.txt
```

Analyze all review comments and identify what needs to change.

Check the PR status, if there is a conflict, you need to resolve it first:

```bash
gh pr status --conflict-status --repo spiriMirror/libuipc
```

## 4. Plan & Implement Fixes

Analyze each review comment, then implement the focused fixes.

Build with the [build command](../cmake-workflow/SKILL.md).

## 5. Test

Follow the same test procedure as [fix-issue step 4](../fix-issue/SKILL.md):

- Run relevant tests to verify the fix. See [run-tests command](../run-tests/SKILL.md).
- Add new tests if the feedback requires new coverage.
- **Do NOT modify existing tests** to make a fix pass.

## 6. Commit & Push

Use the [commit command](../commit/SKILL.md) to format, commit, and push.

The PR will update automatically. Clean up any temporary files:

```bash
rm -rf output/.cursor
```
