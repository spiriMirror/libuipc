# Push Tag

## Overview

Push a new tag to the upstream repository. Optionally create a GitHub release.

## Steps

1. **Verify branch**
   ```bash
   git branch --show-current
   ```
   - Must be on `main` branch. If not, switch: `git checkout main`

2. **Determine tag version**
   ```bash
   git tag --list
   ```
   - If user provides a tag: validate it doesn't exist and is greater than latest
   - If no tag provided: auto-increment patch version (e.g., `v1.2.3` → `v1.2.4`)

3. **Create and push tag**
   ```bash
   git tag <TAG>
   git push origin refs/tags/<TAG>
   ```

4. **Create release (if requested)**
   ```bash
   gh release create <TAG> --repo spiriMirror/libuipc
   ```

**Note:** Always use `--repo spiriMirror/libuipc` to reference the upstream repository.