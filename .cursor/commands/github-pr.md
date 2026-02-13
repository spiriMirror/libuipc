# Create PR

## Overview

Create a well-structured pull request with proper description, labels, and reviewers.

## Steps

Follow the [commit convention](../rules/commit-convention.mdc) for all commit messages and PR titles.

1. **Prepare branch**
   - Ensure all changes are committed using the convention above
   - Format code with [format command](./format.md)
   - Push branch to remote:
     ```bash
     # Use 'origin' remote (most common), or specify a different remote if needed
     git push -u origin HEAD
     ```
   - Verify branch is up to date with main

2. **Create PR**

   Write the PR description to a temp file first to avoid shell escaping issues:

   ```bash
   # Write description to temp file (output/ is gitignored)
   mkdir -p output/.cursor
   cat > output/.cursor/pr_body.md << 'EOF'
   ## Summary
   ...
   EOF
   
   # Create the PR - always target upstream repository
   gh pr create --title "<type>(<scope>): <summary>" --body-file output/.cursor/pr_body.md --base main --repo spiriMirror/libuipc
   
   # Clean up
   rm -r output/.cursor
   ```

   **Note:** Always use `--repo spiriMirror/libuipc` to create PRs against the upstream repository (works with forks).

3. **PR description should include**
   - Summary of changes and motivation
   - Any breaking changes
   - Related issue references (`Fixes #123`)

4. **After creating**
   - Add appropriate labels
   - Assign reviewers if needed
   - Link related issues
