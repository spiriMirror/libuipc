"""Generate Python stubs for the uipc package from the built extension."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import pybind11_stubgen as stubgen


def generate_uipc_stubs(source_dir: Path, output_dir: Path) -> None:
    """Generate the complete ``uipc`` stub tree into ``output_dir``."""
    package_dir = output_dir / "uipc"
    if package_dir.exists():
        for path in package_dir.rglob("*.pyi"):
            path.unlink()

    output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(source_dir))
    stubgen.main(
        [
            "-o",
            str(output_dir),
            "uipc",
            "--ignore-unresolved-names",
            "json",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source_dir", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--build_type", help="retained for xmake/CMake compatibility")
    args = parser.parse_args()
    generate_uipc_stubs(args.source_dir, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
