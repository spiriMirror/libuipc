---
name: fix-issue
description: Fix a GitHub issue with proper branch, testing, and PR workflow
disable-model-invocation: true
---

# Fix Issue

## 1. Identify the Issue

If no issue number is provided, list and ask the developer to pick:

```bash
gh issue list --repo spiriMirror/libuipc
gh issue view <ISSUE_NUMBER> --repo spiriMirror/libuipc
```

**Note:** Always use `--repo spiriMirror/libuipc` to reference the upstream repository (works with forks). See [repository-setup](../repository-setup/SKILL.md) for details.

Determine the issue type: bug fix, feature, performance, refactor, docs, or test improvement.

## 2. Verify the Working Branch

Follow the [owner working agreements](../../../agent_docs/rule.md): work on the current branch.
Do not create a new branch as a workflow default; honor any different
working branch explicitly selected by the owner.

## 3. Plan & Implement

Analyze the issue, identify the root cause, then implement the focused fix.

Build with the [build command](../cmake-workflow/SKILL.md).

## 4. Test

- Test binaries are in `<cmake-binary-directory>/<configuration>/bin/`.
- Tests source code is in `apps/tests/`.
- Run relevant tests to verify the fix. See [run-tests command](../run-tests/SKILL.md).

### Adding New Tests

Add new tests in `apps/tests/` when the bug isn't covered by existing tests.

Rules:
- Tests must be independent of each other.
- Tests must cover the bug / feature.
- **Do NOT modify existing tests** to make a fix pass. If a test itself is wrong, fix it separately and flag it to the developer for review.

## 5. Commit

Use the [commit command](../commit/SKILL.md) to format, commit, and push.

## 6. Review & PR

Review the changes. Create a PR via [github-pr](../github-pr/SKILL.md)
only when requested; targeting `main` requires an explicit owner request.
