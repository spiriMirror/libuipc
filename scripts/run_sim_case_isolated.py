#!/usr/bin/env python3
"""Run every sim_case test in its own process.

The normal `uipc_test_sim_case.exe` run executes all 95 cases in ONE process.
That is the realistic path, but it can hide cross-case pollution of global
state (static caches, driver/allocator state): a bug that only fires after
dozens of engines have run. This runner complements it by executing each
case in a fresh process, so pollution-sensitive failures can be told apart
from case-local ones.

Usage (from the repo root):
    python scripts/run_sim_case_isolated.py                 # all cases
    python scripts/run_sim_case_isolated.py --filter 3*     # glob subset
    python scripts/run_sim_case_isolated.py --shard-count 4 --shard-index 0
    python scripts/run_sim_case_isolated.py --list-only --manifest output/sim-cases.json
    python scripts/run_sim_case_isolated.py --start-from 36_no_surf_but_contact_on
    python scripts/run_sim_case_isolated.py --exe build/Release/bin/uipc_test_sim_case.exe

Exit code is 0 iff every selected case passes.
"""

import argparse
import fnmatch
import json
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_EXE = REPO_ROOT / "build" / "Release" / "bin" / "uipc_test_sim_case.exe"


def list_cases(exe: Path) -> list[str]:
    out = subprocess.run(
        [str(exe), "--list-tests", "--verbosity", "quiet"],
        capture_output=True,
        text=True,
        check=True,
    )
    return [line.strip() for line in out.stdout.splitlines() if line.strip()]


def select_cases(
    cases: list[str],
    pattern: str = "*",
    start_from: str | None = None,
    shard_count: int = 1,
    shard_index: int = 0,
) -> list[str]:
    if shard_count < 1:
        raise ValueError("--shard-count must be at least 1")
    if shard_index < 0 or shard_index >= shard_count:
        raise ValueError(
            "--shard-index must be in [0, --shard-count)"
        )

    selected = sorted(c for c in cases if fnmatch.fnmatchcase(c, pattern))
    selected = selected[shard_index::shard_count]
    if start_from is not None:
        if start_from not in selected:
            raise ValueError(
                f"--start-from case not found in selected shard: {start_from}"
            )
        selected = selected[selected.index(start_from):]

    return selected


def manifest_payload(
    exe: Path,
    source_case_count: int,
    selected: list[str],
    pattern: str,
    start_from: str | None,
    shard_count: int,
    shard_index: int,
) -> dict[str, object]:
    return {
        "schemaVersion": 1,
        "executable": str(exe.resolve()),
        "sourceCaseCount": source_case_count,
        "selectedCaseCount": len(selected),
        "filter": pattern,
        "startFrom": start_from,
        "shard": {
            "count": shard_count,
            "index": shard_index,
            "strategy": "sorted-round-robin",
        },
        "cases": selected,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--exe", type=Path, default=DEFAULT_EXE, help="path to uipc_test_sim_case.exe")
    ap.add_argument("--filter", default="*", help="glob pattern for case names (default: *)")
    ap.add_argument("--start-from", default=None, help="skip cases until this name (inclusive)")
    ap.add_argument("--shard-count", type=int, default=1, help="number of deterministic shards")
    ap.add_argument("--shard-index", type=int, default=0, help="zero-based shard to run")
    ap.add_argument(
        "--list-only", action="store_true", help="print selected cases without running"
    )
    ap.add_argument("--manifest", type=Path, default=None, help="write selected cases as JSON")
    ap.add_argument(
        "--timeout",
        type=int,
        default=900,
        help="per-case timeout in seconds (default: 900)",
    )
    args = ap.parse_args()

    if not args.exe.exists():
        print(f"error: exe not found: {args.exe}", file=sys.stderr)
        return 2

    all_cases = list_cases(args.exe)
    try:
        cases = select_cases(
            all_cases,
            args.filter,
            args.start_from,
            args.shard_count,
            args.shard_index,
        )
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if not cases:
        print("error: no cases selected", file=sys.stderr)
        return 2

    if args.manifest is not None:
        payload = manifest_payload(
            args.exe,
            len(all_cases),
            cases,
            args.filter,
            args.start_from,
            args.shard_count,
            args.shard_index,
        )
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(
            json.dumps(payload, indent=2) + "\n", encoding="utf-8"
        )
        print(f"manifest written to {args.manifest}", file=sys.stderr)

    if args.list_only:
        for case in cases:
            print(case)
        return 0

    print(
        f"running {len(cases)} case(s) in isolated processes "
        f"(shard {args.shard_index}/{args.shard_count})\n"
    )
    failures = []
    t_all = time.time()
    for i, name in enumerate(cases, 1):
        t0 = time.time()
        try:
            r = subprocess.run(
                [str(args.exe), name],
                capture_output=True,
                text=True,
                timeout=args.timeout,
            )
            ok = r.returncode == 0
            tail = ""
        except subprocess.TimeoutExpired:
            ok, tail = False, f"TIMEOUT after {args.timeout}s"
        dt = time.time() - t0
        status = "PASS" if ok else "FAIL"
        print(f"[{i:3d}/{len(cases)}] {status} {dt:7.1f}s  {name}", flush=True)
        if not ok:
            failures.append(name)
            if tail:
                print(f"    {tail}")
            else:
                interesting = [
                    ln for ln in (r.stdout + r.stderr).splitlines()
                    if any(
                        key in ln
                        for key in (
                            "FAILED",
                            "Error",
                            "error",
                            "Exception",
                            "exception",
                            "Line Search",
                        )
                    )
                ]
                for ln in interesting[-10:]:
                    print(f"    {ln}")

    dt_all = time.time() - t_all
    print(f"\n{len(cases) - len(failures)}/{len(cases)} passed in {dt_all / 60:.1f} min")
    if failures:
        print("failed cases:")
        for name in failures:
            print(f"  {name}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
