# Fix PR

## Overview

Fix a pull request based on review feedback comments.

## 1. Identify the PR

If no PR number is provided, list and ask the developer to pick:

```
gh pr list
```

## 2. Checkout the PR Branch

```
gh pr checkout <PR_NUMBER>
```

Or the developer may already be on their own branch for the PR (e.g., from a forked repository).

## 3. Read Review Comments

```
gh pr view <PR_NUMBER> --comments
```

If the output is too long, store it in a temporary file for reference:

```powershell
gh pr view <PR_NUMBER> --comments > pr_comments.tmp
```

Analyze all review comments and identify what needs to change.

## 4. Plan & Implement Fixes

Switch to **plan mode**. Address each review comment, then implement the fixes.

Build with the [build command](./build.md).

## 5. Format

Run [format command](./format.md) on all changed C++ files before committing.

## 6. Test

Follow the same test procedure as [fix-issue step 5](./fix-issue.md#5-test):

- Run relevant tests to verify the fix. See [run-tests command](./run-tests.md).
- Add new tests if the feedback requires new coverage.
- **Do NOT modify existing tests** to make a fix pass.

## 7. Commit & Push

Commit using the [conventional commit convention](./github-pr.md#commit-convention). Then push:

```
git push
```

The PR will update automatically. Clean up any temporary files created in step 3.