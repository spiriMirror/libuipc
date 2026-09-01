#!/usr/bin/env python3
"""Reject wheel CUDA backends that depend on system CUDA Toolkit libraries."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path
from typing import Iterable, Sequence


FORBIDDEN_CUDA_TOOLKIT_LIBRARIES = (
    "cublas",
    "cudart",
    "cufft",
    "curand",
    "cusolver",
    "cusparse",
    "cudnn",
    "nvblas",
    "nvgraph",
    "nvjitlink",
    "nvjpeg",
    "nvrtc",
    "npp",
)


def parse_dependency_names(output: str) -> list[str]:
    names: list[str] = []
    for line in output.splitlines():
        windows = re.fullmatch(r"\s*([A-Za-z0-9_.+-]+\.dll)\s*", line)
        if windows:
            names.append(windows.group(1))
            continue
        elf = re.search(r"Shared library:\s*\[([^]]+)]", line)
        if elf:
            names.append(elf.group(1))
            continue
        objdump = re.fullmatch(r"\s*NEEDED\s+(\S+)\s*", line)
        if objdump:
            names.append(objdump.group(1))
    return names


def forbidden_dependencies(names: Iterable[str]) -> list[str]:
    return sorted(
        {
            name
            for name in names
            if any(token in name.lower() for token in FORBIDDEN_CUDA_TOOLKIT_LIBRARIES)
        }
    )


def _find_dumpbin() -> str:
    executable = shutil.which("dumpbin")
    if executable:
        return executable

    installer = (
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        / "Microsoft Visual Studio"
        / "Installer"
        / "vswhere.exe"
    )
    if installer.is_file():
        result = subprocess.run(
            [
                str(installer),
                "-latest",
                "-products",
                "*",
                "-find",
                r"VC\Tools\MSVC\**\bin\Hostx64\x64\dumpbin.exe",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        matches = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if matches:
            return matches[-1]
    raise RuntimeError("dumpbin.exe was not found")


def dependency_output(binary: Path) -> str:
    if os.name == "nt":
        command = [_find_dumpbin(), "/DEPENDENTS", str(binary)]
    else:
        executable = shutil.which("readelf")
        if executable:
            command = [executable, "-d", str(binary)]
        else:
            executable = shutil.which("objdump")
            if not executable:
                raise RuntimeError("neither readelf nor objdump was found")
            command = [executable, "-p", str(binary)]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return result.stdout


def _wheel_paths(inputs: Sequence[str]) -> list[Path]:
    wheels: list[Path] = []
    for value in inputs:
        path = Path(value)
        if path.is_dir():
            wheels.extend(sorted(path.glob("*.whl")))
        elif path.suffix == ".whl" and path.is_file():
            wheels.append(path)
        else:
            raise FileNotFoundError(f"wheel input does not exist: {path}")
    if not wheels:
        raise FileNotFoundError("no wheel files were found")
    return wheels


def audit_wheel(path: Path) -> list[str]:
    with zipfile.ZipFile(path) as archive:
        members = [
            name
            for name in archive.namelist()
            if Path(name).name.lower() == "uipc_backend_cuda.dll"
            or Path(name).name.lower().startswith("libuipc_backend_cuda.so")
        ]
        if len(members) != 1:
            raise RuntimeError(
                f"{path.name}: expected one CUDA backend binary, found {members}"
            )
        member = members[0]
        with tempfile.TemporaryDirectory(prefix="pyuipc-wheel-audit-") as directory:
            binary = Path(directory) / Path(member).name
            with archive.open(member) as source, binary.open("wb") as destination:
                shutil.copyfileobj(source, destination)
            dependencies = parse_dependency_names(dependency_output(binary))

    forbidden = forbidden_dependencies(dependencies)
    print(f"{path.name}: {', '.join(dependencies) or '(no dynamic dependencies)'}")
    return forbidden


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheels", nargs="+", help="wheel files or directories")
    args = parser.parse_args(argv)

    errors: list[str] = []
    for wheel in _wheel_paths(args.wheels):
        forbidden = audit_wheel(wheel)
        if forbidden:
            errors.append(
                f"{wheel.name}: forbidden system CUDA Toolkit dependencies: "
                + ", ".join(forbidden)
            )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("Wheel CUDA backends require no system CUDA Toolkit libraries.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
