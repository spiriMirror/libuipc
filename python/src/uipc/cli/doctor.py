"""Diagnose Python, wheel, NVIDIA driver, and CUDA runtime compatibility."""

from __future__ import annotations

import argparse
import contextlib
import csv
import ctypes
import importlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable, Sequence


def load_policy() -> dict[str, Any]:
    path = Path(__file__).resolve().parents[1] / "compatibility.json"
    return json.loads(path.read_text(encoding="utf-8"))


def parse_architectures(value: str | Iterable[str]) -> tuple[str, ...]:
    if isinstance(value, str):
        values = re.split(r"[,;\s]+", value)
    else:
        values = [str(item) for item in value]
    return tuple(item.strip().lower() for item in values if item.strip())


def architecture_supports(
    architectures: str | Iterable[str], compute_capability: str
) -> bool | None:
    """Return whether an architecture list can run on a compute capability."""
    tokens = parse_architectures(architectures)
    if not tokens or any(token in {"native", "all", "all-major"} for token in tokens):
        return None

    match = re.fullmatch(r"(\d+)(?:\.(\d+))?", compute_capability.strip())
    if not match:
        return None
    device = int(match.group(1)) * 10 + int(match.group(2) or 0)

    understood = False
    for token in tokens:
        arch_match = re.fullmatch(r"(\d+)(?:-(real|virtual))?", token)
        if not arch_match:
            continue
        understood = True
        target = int(arch_match.group(1))
        kind = arch_match.group(2)
        if kind == "real" and device == target:
            return True
        if kind == "virtual" and device >= target:
            return True
        if kind is None and device >= target:
            # CMake's unsuffixed form emits both SASS and PTX.
            return True
    return False if understood else None


def parse_nvidia_smi_rows(output: str) -> list[dict[str, str]]:
    gpus: list[dict[str, str]] = []
    for row in csv.reader(line for line in output.splitlines() if line.strip()):
        if len(row) < 3:
            continue
        gpus.append(
            {
                "name": row[0].strip(),
                "compute_capability": row[1].strip(),
                "driver_version": row[2].strip(),
            }
        )
    return gpus


def version_at_least(actual: str, required: str) -> bool:
    def components(value: str) -> tuple[int, ...]:
        return tuple(int(item) for item in re.findall(r"\d+", value))

    actual_components = components(actual)
    required_components = components(required)
    width = max(len(actual_components), len(required_components))
    return actual_components + (0,) * (width - len(actual_components)) >= (
        required_components + (0,) * (width - len(required_components))
    )


def query_gpus() -> tuple[list[dict[str, str]], str | None]:
    executable = shutil.which("nvidia-smi")
    if not executable:
        return [], "nvidia-smi was not found on PATH"
    command = [
        executable,
        "--query-gpu=name,compute_cap,driver_version",
        "--format=csv,noheader,nounits",
    ]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError) as error:
        return [], str(error)
    if result.returncode != 0:
        return [], (result.stderr or result.stdout).strip()
    return parse_nvidia_smi_rows(result.stdout), None


def find_cuda_backend(native_dir: Path) -> Path | None:
    patterns = (
        "uipc_backend_cuda.dll",
        "libuipc_backend_cuda.so",
        "libuipc_backend_cuda.so.*",
        "libuipc_backend_cuda.dylib",
    )
    for pattern in patterns:
        matches = sorted(native_dir.glob(pattern))
        if matches:
            return matches[0]
    return None


def load_backend_library(path: Path) -> str | None:
    try:
        with contextlib.ExitStack() as stack:
            if os.name == "nt" and hasattr(os, "add_dll_directory"):
                stack.enter_context(os.add_dll_directory(str(path.parent)))
            ctypes.CDLL(str(path))
    except (OSError, ValueError) as error:
        return str(error)
    return None


def _result(
    name: str,
    status: str,
    detail: str,
    *,
    hint: str | None = None,
    data: Any = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {"name": name, "status": status, "detail": detail}
    if hint:
        result["hint"] = hint
    if data is not None:
        result["data"] = data
    return result


def collect_diagnostics(*, probe_cuda: bool = False) -> dict[str, Any]:
    policy = load_policy()
    python_policy = policy["python"]
    wheel_policy = policy["wheel"]
    checks: list[dict[str, Any]] = []

    current_abi = f"cp{sys.version_info.major}{sys.version_info.minor}"
    supported_abis = tuple(python_policy["abi_tags"])
    python_ok = current_abi in supported_abis and platform.python_implementation() == "CPython"
    checks.append(
        _result(
            "Python",
            "ok" if python_ok else "fail",
            f"{platform.python_implementation()} {platform.python_version()} "
            f"({current_abi}, {platform.machine()})",
            hint=(
                None
                if python_ok
                else f"Use CPython ABIs covered by {python_policy['requires']}: "
                + ", ".join(supported_abis)
            ),
        )
    )

    try:
        uipc = importlib.import_module("uipc")
    except Exception as error:  # pragma: no cover - package import precedes -m uipc
        checks.append(
            _result(
                "pyuipc import",
                "fail",
                f"{type(error).__name__}: {error}",
                hint="Reinstall the wheel matching this Python ABI and platform.",
            )
        )
        return {"policy": policy, "checks": checks}

    package_dir = Path(uipc.__file__).resolve().parent
    native_dir = package_dir / "_native"
    build_info_function = getattr(uipc, "build_info", None)
    build_info = dict(build_info_function()) if callable(build_info_function) else {}
    package_version = str(getattr(uipc, "__version__", "unknown"))
    checks.append(
        _result(
            "pyuipc package",
            "ok" if native_dir.is_dir() else "fail",
            f"version {package_version} at {package_dir}",
            data={"native_dir": str(native_dir), "build_info": build_info},
        )
    )

    built_abi = str(build_info.get("python_abi", "unknown"))
    if built_abi != "unknown":
        checks.append(
            _result(
                "Native ABI",
                "ok" if built_abi == current_abi else "fail",
                f"extension={built_abi}, interpreter={current_abi}",
            )
        )

    backend = find_cuda_backend(native_dir)
    checks.append(
        _result(
            "CUDA backend",
            "ok" if backend else "fail",
            str(backend) if backend else f"not found under {native_dir}",
            hint=None if backend else "Install a CUDA-enabled pyuipc wheel or rebuild with CUDA.",
        )
    )

    compiled_toolkit = str(build_info.get("cuda_toolkit_version", "unknown"))
    checks.append(
        _result(
            "CUDA runtime",
            "ok",
            f"self-contained runtime built with CUDA {compiled_toolkit}; "
            "no system CUDA Toolkit is required",
            data={"compiled_toolkit": compiled_toolkit, "system_toolkit": False},
        )
    )

    if backend:
        load_error = load_backend_library(backend)
        checks.append(
            _result(
                "CUDA backend load",
                "fail" if load_error else "ok",
                load_error or "backend and its dynamic dependencies loaded",
                hint=(
                    "Check the CUDA runtime result and dynamic-library search path."
                    if load_error
                    else None
                ),
            )
        )

    gpus, gpu_error = query_gpus()
    driver_platform = "windows" if os.name == "nt" else "linux"
    minimum_driver = str(wheel_policy["minimum_driver"][driver_platform])
    driver_compatible = bool(gpus) and all(
        version_at_least(gpu["driver_version"], minimum_driver) for gpu in gpus
    )
    checks.append(
        _result(
            "NVIDIA driver",
            (
                "ok"
                if driver_compatible
                else "fail"
                if gpus or probe_cuda
                else "warn"
            ),
            (
                "; ".join(
                    f"{gpu['name']} (sm_{gpu['compute_capability'].replace('.', '')}, "
                    f"driver {gpu['driver_version']})"
                    for gpu in gpus
                )
                if gpus
                else gpu_error or "no NVIDIA GPU reported"
            ),
            hint=(
                None
                if driver_compatible
                else f"Install NVIDIA driver {minimum_driver} or newer and verify "
                "nvidia-smi. A local CUDA Toolkit is not required."
            ),
            data=gpus,
        )
    )

    compiled_architectures = str(build_info.get("cuda_architectures", "unknown"))
    architecture_source = "native build metadata"
    for index, gpu in enumerate(gpus):
        supported = (
            None
            if compiled_architectures in {"", "unknown", "none"}
            else architecture_supports(
                compiled_architectures, gpu["compute_capability"]
            )
        )
        status = "ok" if supported else ("fail" if supported is False else "warn")
        checks.append(
            _result(
                f"GPU architecture #{index}",
                status,
                f"sm_{gpu['compute_capability'].replace('.', '')} against "
                f"[{compiled_architectures}] ({architecture_source})",
                hint=(
                    (
                        "A native architecture target cannot be compared statically; use "
                        "--probe-cuda on this machine."
                        if compiled_architectures == "native"
                        else "This package predates native build metadata; reinstall a "
                        "newer wheel or inspect it with cuobjdump."
                    )
                    if supported is None
                    else None
                    if supported
                    else "Install a wheel containing this SASS target or a compatible PTX target, "
                    "or build from source with -DUIPC_CUDA_ARCHITECTURES=native."
                ),
            )
        )

    if probe_cuda:
        try:
            with tempfile.TemporaryDirectory(prefix="pyuipc-doctor-") as workspace:
                engine = uipc.Engine("cuda", workspace)
                del engine
        except Exception as error:
            checks.append(
                _result(
                    "CUDA engine probe",
                    "fail",
                    f"{type(error).__name__}: {error}",
                    hint="Review the failed runtime, backend-load, and architecture checks above.",
                )
            )
        else:
            for check in checks:
                if (
                    check["name"].startswith("GPU architecture #")
                    and check["status"] == "warn"
                ):
                    check["status"] = "ok"
                    check["detail"] += "; verified by CUDA engine probe"
                    check.pop("hint", None)
            checks.append(
                _result(
                    "CUDA engine probe",
                    "ok",
                    "Engine('cuda') constructed and destroyed successfully",
                )
            )

    return {"policy": policy, "checks": checks}


def print_report(report: dict[str, Any]) -> None:
    print("Libuipc compatibility doctor")
    print("=" * 60)
    for check in report["checks"]:
        print(f"[{check['status'].upper():4}] {check['name']}: {check['detail']}")
        if check.get("hint"):
            print(f"       Hint: {check['hint']}")


def main(argv: Sequence[str] | None = None) -> int:
    """Run compatibility checks for the installed pyuipc package."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--probe-cuda",
        action="store_true",
        help="construct Engine('cuda') after non-invasive checks",
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument(
        "--strict", action="store_true", help="return nonzero for warnings as well as failures"
    )
    args = parser.parse_args(argv)
    if args.json and args.probe_cuda:
        parser.error(
            "--json and --probe-cuda cannot be combined because native backend logs "
            "would corrupt the JSON stream"
        )

    report = collect_diagnostics(probe_cuda=args.probe_cuda)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_report(report)

    statuses = {check["status"] for check in report["checks"]}
    if "fail" in statuses or (args.strict and "warn" in statuses):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
