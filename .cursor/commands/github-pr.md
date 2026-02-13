# Create PR

## Overview

Create a well-structured pull request with proper description, labels, and reviewers.

## Steps

Follow the [commit convention](../rules/commit-convention.mdc) for all commit messages and PR titles.

1. **Prepare branch**
   - Ensure all changes are committed using the convention above
   - Format code with [format command](./format.md)
   - Push branch to remote: `git push -u origin HEAD`
   - Verify branch is up to date with main

2. **Create PR**

   Write the PR description to a temp file first to avoid shell escaping issues:

   ```bash
   # Write description to temp file (output/ is gitignored)
   mkdir -p output/.cursor
   # Write the PR body in markdown to the temp file
   cat > output/.cursor/pr_body.md << 'EOF'
   ## Summary
   ...
   EOF
   # Create the PR using the temp file
   gh pr create --title "<type>(<scope>): <summary>" --body-file output/.cursor/pr_body.md --base main
   # Clean up
   rm -r output/.cursor
   ```

3. **PR description should include**
   - Summary of changes and motivation
   - Any breaking changes
   - Related issue references (`Fixes #123`)

4. **After creating**
   - Add appropriate labels
   - Assign reviewers if needed
   - Link related issues
