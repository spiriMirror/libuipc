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


def _version_tuple(value: object) -> tuple[int, ...]:
    return tuple(int(item) for item in re.findall(r"\d+", str(value)))


def check_policy(root: Path = ROOT) -> list[str]:
    policy = load_policy(root / "python" / "src" / "uipc" / "compatibility.json")
    python_policy = policy["python"]
    wheel_policy = policy["wheel"]
    errors: list[str] = []

    minimum_driver = wheel_policy.get("minimum_driver")
    minimum_ptx_driver = wheel_policy.get("minimum_ptx_jit_driver")
    for name, values in (
        ("minimum_driver", minimum_driver),
        ("minimum_ptx_jit_driver", minimum_ptx_driver),
    ):
        if not isinstance(values, dict):
            errors.append(f"compatibility.json: wheel.{name} must be an object")
            continue
        for driver_platform in ("linux", "windows"):
            if not values.get(driver_platform):
                errors.append(
                    f"compatibility.json: wheel.{name} is missing {driver_platform!r}"
                )

    if isinstance(minimum_driver, dict) and isinstance(minimum_ptx_driver, dict):
        for driver_platform in ("linux", "windows"):
            base = minimum_driver.get(driver_platform)
            ptx = minimum_ptx_driver.get(driver_platform)
            if base and ptx and _version_tuple(ptx) < _version_tuple(base):
                errors.append(
                    "compatibility.json: PTX JIT driver floor "
                    f"{ptx!r} is older than the base {driver_platform} floor {base!r}"
                )

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
        if wheel_policy.get("requires_system_cuda_toolkit", True):
            expected_runtime_text = f"require CUDA {wheel_policy['cuda_toolkit']}"
        else:
            expected_runtime_text = "require a compatible NVIDIA driver"
        if expected_runtime_text not in project.get("description", ""):
            errors.append(
                f"{relative_path}: description must state that prebuilt wheels "
                f"{expected_runtime_text}"
            )

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
