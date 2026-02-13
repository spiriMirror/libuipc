# Create PR

## Overview

Create a well-structured pull request with proper description, labels, and reviewers.

## Commit Convention

Use [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <short summary>
```

### Types

| Type | When to Use |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `refactor` | Code restructuring (no behavior change) |
| `perf` | Performance improvement |
| `test` | Adding or updating tests |
| `docs` | Documentation only |
| `build` | Build system or dependency changes |
| `ci` | CI/CD changes |
| `chore` | Other maintenance tasks |

### Examples

```
feat(geometry): add label_open_edge utility
fix(cuda): resolve race condition in contact solver
refactor(core): simplify scene validation logic
test(sim_case): add ABD-FEM contact test
docs: update build instructions
```

### Rules

- **Scope** is optional but encouraged — use the module name (e.g., `geometry`, `core`, `cuda`, `io`).
- **Summary** should be lowercase, imperative mood, no period at the end.
- Add a blank line then a body for longer explanations if needed.
- Reference issues with `Fixes #<number>` or `Closes #<number>` in the body.

## Steps

1. **Prepare branch**
   - Ensure all changes are committed using the convention above
   - Format code with [format command](./format.md)
   - Push branch to remote: `git push -u origin HEAD`
   - Verify branch is up to date with main

2. **Create PR**
   ```powershell
   gh pr create --title "<type>(<scope>): <summary>" --body "<description>" --base main
   ```

3. **PR description should include**
   - Summary of changes and motivation
   - Any breaking changes
   - Related issue references (`Fixes #123`)

4. **After creating**
   - Add appropriate labels
   - Assign reviewers if needed
   - Link related issues
