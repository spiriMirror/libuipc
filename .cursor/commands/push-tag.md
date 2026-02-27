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
   - **Generate structured release notes** (do not use `--generate-notes`):
     1. Get commits between previous tag and `<TAG>`:
        ```bash
        git log <PREV_TAG>..<TAG> --pretty=format:'%s' --no-merges
        ```
     2. **Summarize and structure** the changes into clear sections, e.g.:
        - **Features** – new functionality
        - **Fixes** – bug fixes
        - **Documentation** – docs, examples, guides
        - **Refactor / Chore** – internal changes, deps, tooling
     3. Use concise, user-facing language. Group related changes. Avoid raw commit hashes or jargon unless helpful.
     4. Write notes to a temp file (e.g. `release_notes.md`) and create the release:
        ```bash
        gh release create <TAG> --repo spiriMirror/libuipc --notes-file release_notes.md
        rm release_notes.md
        ```

**Note:** Always use `--repo spiriMirror/libuipc` to reference the upstream repository.