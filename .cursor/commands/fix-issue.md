# Fix Issue

## 1. Identify the Issue

If no issue number is provided, list and ask the developer to pick:

```
gh issue list
gh issue view <ISSUE_NUMBER>
```

Determine the issue type: bug fix, feature, performance, refactor, docs, or test improvement.

## 2. Create a Branch

```
git checkout -b fix/issue-<ISSUE_NUMBER>
```

Use `feat/` prefix for features, `fix/` for bugs, `refactor/` for refactoring, etc.

## 3. Plan & Implement

Switch to **plan mode**. Analyze the issue, identify root cause, then implement the fix.

Build with the [build command](./build.md).

## 4. Format

Run [format command](./format.md) on all changed C++ files before committing.

## 5. Test

- Test binaries are in `<cmake-binary-directory>/<configuration>/bin/`.
- Tests source code is in `apps/tests/`.
- Run relevant tests to verify the fix. See [run-tests command](./run-tests.md).

### Adding New Tests

Add new tests in `apps/tests/` when the bug isn't covered by existing tests.

Rules:
- Tests must be independent of each other.
- Tests must cover the bug / feature.
- **Do NOT modify existing tests** to make a fix pass. If a test itself is wrong, fix it separately and flag it to the developer for review.

## 6. Review & PR

Switch to **review mode**, review all changes, then create a PR via [github-pr.md](./github-pr.md).
