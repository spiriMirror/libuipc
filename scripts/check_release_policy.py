#!/usr/bin/env python3
"""Check that packaging and CI mirror the canonical wheel support policy."""

from __future__ import annotations

import json
import re
import tomllib
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = ROOT / "python" / "src" / "uipc" / "compatibility.json"


def load_policy(path: Path = POLICY_PATH) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as stream:
        return tomllib.load(stream)


def check_policy(root: Path = ROOT) -> list[str]:
    policy = load_policy(root / "python" / "src" / "uipc" / "compatibility.json")
    python_policy = policy["python"]
    wheel_policy = policy["wheel"]
    errors: list[str] = []

    for relative_path in ("pyproject.toml", "python/pyproject.toml"):
        path = root / relative_path
        project = _load_toml(path)["project"]
        if project["requires-python"] != python_policy["requires"]:
            errors.append(
                f"{relative_path}: requires-python is "
                f"{project['requires-python']!r}, expected "
                f"{python_policy['requires']!r}"
            )
        classifiers = set(project.get("classifiers", []))
        for abi_tag in python_policy["abi_tags"]:
            version = f"{abi_tag[2]}.{abi_tag[3:]}"
            classifier = f"Programming Language :: Python :: {version}"
            if classifier not in classifiers:
                errors.append(f"{relative_path}: missing classifier {classifier!r}")

    root_pyproject = _load_toml(root / "pyproject.toml")
    actual_architectures = root_pyproject["tool"]["scikit-build"]["cmake"][
        "define"
    ]["UIPC_CUDA_ARCHITECTURES"].split(";")
    if actual_architectures != wheel_policy["cuda_architectures"]:
        errors.append(
            "pyproject.toml: UIPC_CUDA_ARCHITECTURES is "
            f"{actual_architectures!r}, expected "
            f"{wheel_policy['cuda_architectures']!r}"
        )

    workflow_path = root / ".github" / "workflows" / "python-wheels.yml"
    workflow = workflow_path.read_text(encoding="utf-8")
    workflow_tags = re.findall(r'^\s+- "(cp\d+)"\s*$', workflow, re.MULTILINE)
    if workflow_tags != python_policy["abi_tags"]:
        errors.append(
            ".github/workflows/python-wheels.yml: Python matrix is "
            f"{workflow_tags!r}, expected {python_policy['abi_tags']!r}"
        )

    workflow_cuda_versions = re.findall(
        r"^\s+- (\d+\.\d+(?:\.\d+)?)\s*$", workflow, re.MULTILINE
    )
    expected_cuda = str(wheel_policy["cuda_toolkit"])
    if len(workflow_cuda_versions) != 1 or not workflow_cuda_versions[0].startswith(
        f"{expected_cuda}."
    ):
        errors.append(
            ".github/workflows/python-wheels.yml: CUDA matrix is "
            f"{workflow_cuda_versions!r}, expected one {expected_cuda}.x entry"
        )

    return errors


def main() -> int:
    errors = check_policy()
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print(f"Release policy mirrors are synchronized with {POLICY_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
