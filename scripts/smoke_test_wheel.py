#!/usr/bin/env python3
"""Smoke-test an installed pyuipc wheel without requiring a GPU."""

from __future__ import annotations

import json
import os
import sys
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

    policy_path = Path(uipc.__file__).resolve().parent / "compatibility.json"
    if not policy_path.is_file():
        raise RuntimeError(f"compatibility policy is missing: {policy_path}")
    policy = json.loads(policy_path.read_text(encoding="utf-8"))

    build_info = dict(uipc.build_info())
    expected_abi = f"cp{sys.version_info.major}{sys.version_info.minor}"
    if build_info["python_abi"] != expected_abi:
        raise RuntimeError(
            f"native ABI {build_info['python_abi']!r} does not match {expected_abi!r}"
        )
    if not build_info["cuda_backend"]:
        raise RuntimeError("published wheel was built without the CUDA backend")
    actual_architectures = str(build_info["cuda_architectures"]).split(",")
    expected_architectures = policy["wheel"]["cuda_architectures"]
    if actual_architectures != expected_architectures:
        raise RuntimeError(
            f"wheel CUDA architectures {actual_architectures!r} do not match "
            f"release policy {expected_architectures!r}"
        )
    if not str(build_info["cuda_toolkit_version"]).startswith(
        str(policy["wheel"]["cuda_toolkit"])
    ):
        raise RuntimeError(
            f"wheel CUDA toolkit {build_info['cuda_toolkit_version']!r} does not match "
            f"release policy {policy['wheel']['cuda_toolkit']!r}"
        )

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
                "build_info": build_info,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
