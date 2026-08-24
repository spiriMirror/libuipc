#!/usr/bin/env python3
"""Smoke-test an installed pyuipc wheel without requiring a GPU."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path


def main() -> int:
    import uipc
    from uipc import Engine

    version = str(uipc.__version__)
    expected_version = os.environ.get("UIPC_EXPECTED_VERSION")
    if expected_version and version != expected_version:
        raise RuntimeError(
            f"installed pyuipc version {version!r} does not match "
            f"expected version {expected_version!r}"
        )

    native_dir = Path(uipc.__file__).resolve().parent / "_native"
    if not native_dir.is_dir():
        raise RuntimeError(f"native module directory is missing: {native_dir}")

    with tempfile.TemporaryDirectory(prefix="pyuipc-wheel-smoke-") as workspace:
        engine = Engine("none", workspace)
        del engine

    print(
        json.dumps(
            {
                "version": version,
                "package": str(Path(uipc.__file__).resolve()),
                "native_dir": str(native_dir),
                "backend": "none",
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
