# Fix PR

## Overview

Fix a pull request based on review feedback comments.

## 1. Identify the PR

If no PR number is provided, list and ask the developer to pick:

```bash
gh pr list --repo spiriMirror/libuipc
```

**Note:** Always use `--repo spiriMirror/libuipc` to reference the upstream repository (works with forks).

## 2. Checkout the PR Branch

```bash
gh pr checkout <PR_NUMBER> --repo spiriMirror/libuipc
```

Or the developer may already be on their own branch for the PR (e.g., from a forked repository).

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

Switch to **plan mode**. Address each review comment, then implement the fixes.

Build with the [build command](./build.md).

## 5. Test

Follow the same test procedure as [fix-issue step 4](./fix-issue.md):

- Run relevant tests to verify the fix. See [run-tests command](./run-tests.md).
- Add new tests if the feedback requires new coverage.
- **Do NOT modify existing tests** to make a fix pass.

## 6. Commit & Push

Use the [commit command](./commit.md) to format, commit, and push.

The PR will update automatically. Clean up any temporary files:

```bash
rm -rf output/.cursor
```
