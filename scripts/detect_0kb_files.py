#!/usr/bin/env python3
"""Find zero-byte source files and optionally fail as a repository gate."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCAN_ROOTS = ("include", "src")


def detect_0kb_files(
    project_root: Path = ROOT,
    scan_roots: Sequence[str] = DEFAULT_SCAN_ROOTS,
) -> list[Path]:
    """Return zero-byte files below the selected roots in stable order."""

    files: list[Path] = []
    for relative_root in scan_roots:
        root = project_root / relative_root
        if not root.exists():
            continue
        files.extend(
            path
            for path in root.rglob("*")
            if path.is_file() and path.stat().st_size == 0
        )
    return sorted(files, key=lambda path: path.as_posix())


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="return a nonzero status when a zero-byte file is found",
    )
    args = parser.parse_args(argv)

    files = detect_0kb_files()
    for path in files:
        print(path.relative_to(ROOT).as_posix())

    if files:
        print(f"Found {len(files)} zero-byte source file(s).")
    else:
        print("No zero-byte source files found.")
    return int(args.check and bool(files))


if __name__ == "__main__":
    raise SystemExit(main())
