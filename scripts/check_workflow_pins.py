#!/usr/bin/env python3
"""Require immutable, reviewable GitHub Action and vcpkg workflow pins."""

from __future__ import annotations

import re
from collections.abc import Sequence
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKFLOW_ROOT = Path(".github/workflows")
COMMIT_RE = re.compile(r"[0-9a-f]{40}")
SYMBOLIC_REVISION_RE = re.compile(
    r"^\s*revision:\s*['\"]?(?:HEAD|latest|main|master)['\"]?\s*(?:#.*)?$",
    re.IGNORECASE,
)


def check_workflow_pins(project_root: Path = ROOT) -> list[str]:
    errors: list[str] = []
    workflow_root = project_root / WORKFLOW_ROOT
    paths = sorted(
        (*workflow_root.glob("*.yml"), *workflow_root.glob("*.yaml")),
        key=lambda path: path.as_posix(),
    )

    for path in paths:
        relative_path = path.relative_to(project_root).as_posix()
        lines = path.read_text(encoding="utf-8").splitlines()
        for line_number, line in enumerate(lines, 1):
            stripped = line.strip()
            if stripped.startswith("uses:") or stripped.startswith("- uses:"):
                value = stripped.split("uses:", maxsplit=1)[1].strip()
                if value.startswith("./") or value.startswith("docker://"):
                    continue
                action_ref, separator, remainder = value.partition("@")
                if not separator:
                    errors.append(
                        f"{relative_path}:{line_number}: action has no ref: {value}"
                    )
                    continue
                ref, _, comment = remainder.partition("#")
                ref = ref.strip()
                if not COMMIT_RE.fullmatch(ref):
                    errors.append(
                        f"{relative_path}:{line_number}: {action_ref} uses "
                        f"mutable ref {ref!r}"
                    )
                if not comment.strip():
                    errors.append(
                        f"{relative_path}:{line_number}: pinned action needs a "
                        "reviewed tag comment"
                    )

            if SYMBOLIC_REVISION_RE.fullmatch(line):
                errors.append(
                    f"{relative_path}:{line_number}: revision must be an "
                    "immutable commit"
                )

    return errors


def main(argv: Sequence[str] | None = None) -> int:
    del argv
    errors = check_workflow_pins()
    if errors:
        print("Workflow pin check failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("Workflow pin check passed: all external actions use reviewed commit SHAs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
