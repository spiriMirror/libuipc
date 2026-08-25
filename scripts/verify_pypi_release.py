#!/usr/bin/env python3
"""Wait for a PyPI release and verify its expected wheel matrix."""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any


POLICY_PATH = (
    Path(__file__).resolve().parents[1]
    / "python"
    / "src"
    / "uipc"
    / "compatibility.json"
)
with POLICY_PATH.open(encoding="utf-8") as policy_stream:
    _POLICY = json.load(policy_stream)

DEFAULT_PYTHON_TAGS = tuple(_POLICY["python"]["abi_tags"])
DEFAULT_PLATFORM_TOKENS = tuple(_POLICY["wheel"]["platform_tokens"])


def validate_release(
    payload: Mapping[str, Any],
    package: str,
    version: str,
    python_tags: Iterable[str] = DEFAULT_PYTHON_TAGS,
    platform_tokens: Iterable[str] = DEFAULT_PLATFORM_TOKENS,
) -> list[str]:
    python_tags = tuple(python_tags)
    platform_tokens = tuple(platform_tokens)
    actual_version = str(payload.get("info", {}).get("version", ""))
    if actual_version != version:
        raise ValueError(
            f"Package index returned version {actual_version!r} for {package}, "
            f"expected {version!r}"
        )

    wheel_names = sorted(
        str(item.get("filename", ""))
        for item in payload.get("urls", [])
        if str(item.get("filename", "")).endswith(".whl")
    )

    missing: list[str] = []
    for python_tag in python_tags:
        abi_marker = f"-{python_tag}-{python_tag}-"
        for platform_token in platform_tokens:
            if not any(
                abi_marker in filename and platform_token in filename
                for filename in wheel_names
            ):
                missing.append(f"{python_tag}/{platform_token}")

    if missing:
        raise ValueError(
            f"Package release {package}=={version} is missing wheel variants: "
            + ", ".join(missing)
        )

    expected_count = len(python_tags) * len(platform_tokens)
    if len(wheel_names) != expected_count:
        raise ValueError(
            f"Package release {package}=={version} has {len(wheel_names)} wheels; "
            f"expected exactly {expected_count}: {wheel_names}"
        )

    return wheel_names


def fetch_release(
    repository_url: str, package: str, version: str
) -> Mapping[str, Any]:
    package_path = urllib.parse.quote(package, safe="")
    version_path = urllib.parse.quote(version, safe="")
    url = (
        f"{repository_url.rstrip('/')}/pypi/"
        f"{package_path}/{version_path}/json"
    )
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json", "User-Agent": "libuipc-release-check"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def wait_for_release(
    repository_url: str,
    package: str,
    version: str,
    timeout: float,
    poll_interval: float,
    python_tags: Iterable[str] = DEFAULT_PYTHON_TAGS,
    platform_tokens: Iterable[str] = DEFAULT_PLATFORM_TOKENS,
) -> list[str]:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None

    while True:
        try:
            payload = fetch_release(repository_url, package, version)
            return validate_release(
                payload,
                package,
                version,
                python_tags=python_tags,
                platform_tokens=platform_tokens,
            )
        except (OSError, ValueError, urllib.error.HTTPError) as error:
            last_error = error
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Package release {package}=={version} did not become complete "
                    f"within {timeout:g}s"
                ) from last_error
            print(f"Waiting for package-index propagation: {error}", flush=True)
            time.sleep(poll_interval)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository-url", default="https://pypi.org")
    parser.add_argument("--package", default="pyuipc")
    parser.add_argument("--version", required=True)
    parser.add_argument("--python-tags", nargs="+", default=DEFAULT_PYTHON_TAGS)
    parser.add_argument(
        "--platform-tokens", nargs="+", default=DEFAULT_PLATFORM_TOKENS
    )
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--poll-interval", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    wheels = wait_for_release(
        args.repository_url,
        args.package,
        args.version,
        timeout=args.timeout,
        poll_interval=args.poll_interval,
        python_tags=args.python_tags,
        platform_tokens=args.platform_tokens,
    )
    print(
        f"Verified {args.package}=={args.version} on "
        f"{args.repository_url.rstrip('/')}:")
    for wheel in wheels:
        print(f"  {wheel}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
