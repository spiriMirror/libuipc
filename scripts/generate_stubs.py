#!/usr/bin/env python3
"""
Standalone script to generate type stubs for pyuipc.
"""

import sys
import shutil
import argparse as ap
import pathlib
from mypy import stubgen
import optional_import  # help stubgen to detect optional modules' api


def generate_stubs(target_dir, output_dir=None):
    """
    Generate type stubs for pyuipc package.
    
    Args:
        target_dir: Directory where pyuipc module is located (for importing)
        output_dir: Directory where stubs should be written (optional)
    """
    optional_import.EnabledModules.report()
    PACKAGE_NAME = 'pyuipc'
    
    if output_dir:
        typings_dir = pathlib.Path(output_dir)
    else:
        # Default: output to target_dir's parent / src
        typings_dir = pathlib.Path(target_dir).parent.parent / 'src'
    
    # clear the typings directory
    typings_folder = typings_dir / PACKAGE_NAME
    print(f'Clear typings directory: {typings_folder}')
    shutil.rmtree(typings_folder, ignore_errors=True)

    # generate the stubs
    print(f'Try generating stubs to {typings_dir}')
    sys.path.append(str(target_dir))
    
    options = stubgen.Options(
        pyversion=sys.version_info[:2],
        no_import=False,
        inspect=True,
        doc_dir='',
        search_path=[str(target_dir)],
        interpreter=sys.executable,
        parse_only=False,
        ignore_errors=False,
        include_private=False,
        output_dir=str(typings_dir),
        modules=[],
        packages=[PACKAGE_NAME],
        files=[],
        verbose=True,
        quiet=False,
        export_less=False,
        include_docstrings=True
    )
    
    typings_dir.mkdir(parents=True, exist_ok=True)
    success = False
    try:
        stubgen.generate_stubs(options)
        success = True
    except Exception as e:
        print(f'Error generating stubs: {e}')
    return success

def main():
    parser = ap.ArgumentParser(
        description='Generate type stubs for pyuipc package.'
    )
    parser.add_argument(
        '--target-dir',
        required=True,
        help='Directory where pyuipc module is located (for importing)'
    )
    parser.add_argument(
        '--output-dir',
        default=None,
        help='Directory where stubs should be written (optional, defaults to target_dir/../src)'
    )
    
    args = parser.parse_args()
    
    target_dir = pathlib.Path(args.target_dir)
    if not target_dir.exists():
        print(f'Error: target directory does not exist: {target_dir}')
        sys.exit(1)
    
    print(f'Generating stubs for pyuipc from {target_dir}')
    success = generate_stubs(target_dir, args.output_dir)
    if not success:
        print(f'Error: failed to generate stubs')
        sys.exit(1)

    print('Stub generation completed successfully')


if __name__ == '__main__':
    main()

