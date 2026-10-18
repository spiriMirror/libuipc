"""Identify a benchmark's Python package and colocated native binaries, without CUDA work."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def collect_runtime(module: Any = None) -> dict[str, Any]:
    if module is None:
        import uipc as module

    package = Path(module.__file__).resolve().parent
    native = package / "_native"
    binaries = [
        path for path in native.iterdir()
        if path.is_file()
        and (path.suffix.lower() in {".dll", ".pyd", ".so", ".dylib"}
             or ".so." in path.name)
    ]
    return {
        "pythonExecutable": str(Path(sys.executable).resolve()),
        "packagePath": str(package),
        "buildInfo": module.build_info(),
        "nativeFiles": [
            {"path": str(path.resolve()), "sizeBytes": path.stat().st_size,
             "sha256": file_sha256(path)}
            for path in sorted(binaries)
        ],
        "pythonSources": [
            {"path": str(path.relative_to(package)), "sha256": file_sha256(path)}
            for path in sorted(package.rglob("*.py"))
        ],
    }


if __name__ == "__main__":
    # Match a sample's import search directory, not this helper's scripts/ dir.
    sys.path[0] = os.getcwd()
    print(json.dumps(collect_runtime(), sort_keys=True))
