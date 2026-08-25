"""Command dispatcher for ``python -m uipc`` and the ``uipc`` console script."""

from __future__ import annotations

import importlib
import sys
from collections.abc import Sequence


COMMANDS = {
    "benchmark": "uipc.cli.benchmark",
    "config-schema": "uipc.cli.config_schema",
    "doctor": "uipc.cli.doctor",
    "mesh-doctor": "uipc.cli.mesh_doctor",
    "uid-info": "uipc.cli.uid_info",
}


def _print_help() -> None:
    print("Usage: python -m uipc <command> [options]")
    print()
    print("Commands:")
    for command in COMMANDS:
        print(f"  {command}")
    print()
    print("Run `python -m uipc <command> --help` for command-specific help.")


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        _print_help()
        return 0

    command = args.pop(0)
    module_name = COMMANDS.get(command)
    if module_name is None:
        print(f"Unknown command: {command}", file=sys.stderr)
        _print_help()
        return 2

    module = importlib.import_module(module_name)
    command_main = getattr(module, "main")
    if command in {"config-schema", "doctor"}:
        return int(command_main(args))

    previous_argv = sys.argv
    try:
        sys.argv = [f"uipc {command}", *args]
        result = command_main()
    finally:
        sys.argv = previous_argv
    return int(result or 0)


if __name__ == "__main__":
    raise SystemExit(main())
