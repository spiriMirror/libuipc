#!/usr/bin/env python3
"""Format (or check) the C++ files changed vs a base ref, exactly like CI.

The clang-format workflow checks files changed against the merge base with
`clang-format-18 --dry-run --Werror`. This script applies the same file
selection locally and either reformats them in place (default) or runs the
CI check verbatim (--check).

Usage:
    python scripts/format_changed.py                # format in place
    python scripts/format_changed.py --check        # CI-equivalent dry run
    python scripts/format_changed.py --base origin/refactor-main

Formatter resolution order: $CLANG_FORMAT, `clang-format-18` on PATH,
`clang-format` on PATH, then the repo-local pinned copy at
output/venv_clangfmt/Scripts/clang-format.exe (CI runs 18.x; use a matching
major version or the check may disagree).
"""
import argparse
import os
import shutil
import subprocess
import sys

EXTENSIONS = ('*.h', '*.hpp', '*.cpp', '*.inl', '*.cu', '*.cuh')
LOCAL_VENV = os.path.join('output', 'venv_clangfmt', 'Scripts',
                          'clang-format.exe')


def find_formatter():
    for cand in (os.environ.get('CLANG_FORMAT'), 'clang-format-18',
                 'clang-format', LOCAL_VENV):
        if not cand:
            continue
        if os.path.sep in cand or '/' in cand:
            if os.path.isfile(cand):
                return cand
        elif shutil.which(cand):
            return shutil.which(cand)
    sys.exit('clang-format not found (set $CLANG_FORMAT or install '
             'clang-format-18)')


def changed_files(base):
    out = subprocess.run(
        ['git', 'diff', '--name-only', '--diff-filter=ACMR',
         f'{base}...HEAD', '--'] + list(EXTENSIONS),
        capture_output=True, text=True, check=True).stdout
    return [f for f in out.splitlines() if f.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check',
                    action='store_true',
                    help='dry-run like CI instead of formatting in place')
    ap.add_argument('--base', default='origin/main')
    args = ap.parse_args()

    fmt = find_formatter()
    files = changed_files(args.base)
    if not files:
        print('no changed C++ files vs', args.base)
        return 0

    version = subprocess.run([fmt, '--version'], capture_output=True,
                             text=True).stdout.strip()
    print(f'{fmt} ({version})')

    if args.check:
        cmd = [fmt, '--dry-run', '--Werror'] + files
    else:
        cmd = [fmt, '-i'] + files
        print(f'formatting {len(files)} file(s)...')
    result = subprocess.run(cmd)
    if args.check and result.returncode != 0:
        print('clang-format check FAILED — run without --check to fix')
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
