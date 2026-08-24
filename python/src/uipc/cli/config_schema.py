"""Inspect the machine-readable libuipc scene-configuration contract."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence

from uipc import Scene


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "key",
        nargs="?",
        help="optional slash-separated key, for example contact/d_hat",
    )
    parser.add_argument(
        "--compact", action="store_true", help="emit compact rather than indented JSON"
    )
    args = parser.parse_args(argv)

    schema = Scene.config_schema()
    payload = schema
    if args.key:
        try:
            payload = schema["entries"][args.key]
        except KeyError:
            parser.error(f"unknown scene configuration key: {args.key}")

    print(
        json.dumps(
            payload,
            indent=None if args.compact else 2,
            separators=(",", ":") if args.compact else None,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
