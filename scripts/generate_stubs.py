#!/usr/bin/env python3
"""
Standalone script to generate type stubs for pyuipc.
"""

import sys
import shutil
import argparse as ap
import pathlib
from mypy import stubgen
import build_utils
import optional_import  # help stubgen to detect optional modules' api

def generate_stubs(target_dir: str, stub_output: str):
    """
    Generate type stubs for pyuipc package.
    
    Args:
        target_dir: Directory where pyuipc module is located (for importing)
        output_dir: Directory where stubs should be written (optional)
    """
    optional_import.EnabledModules.report()
    PACKAGE_NAME = 'pyuipc'
    
    typings_folder = pathlib.Path(stub_output) / PACKAGE_NAME

    print(f'Clear typings folder: {typings_folder}')
    shutil.rmtree(typings_folder, ignore_errors=True)

    # generate the stubs
    print(f'Try generating stubs to {typings_folder}')
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
        output_dir=str(typings_folder),
        modules=[],
        packages=[PACKAGE_NAME],
        files=[],
        verbose=True,
        quiet=False,
        export_less=False,
        include_docstrings=True
    )
    
    typings_folder.mkdir(parents=True, exist_ok=True)
    success = False
    try:
        stubgen.generate_stubs(options)
        success = True
    except Exception as e:
        print(f'Error generating stubs: {e}')
    return success

def main():
    args = ap.ArgumentParser(description='Copy the release directory to the project directory')
    args.add_argument('--target', help='target pyuipc shared library', required=True)
    args.add_argument('--binary_dir', help='CMAKE_BINARY_DIR', required=True)
    args.add_argument('--config', help='$<CONFIG>', required=True)
    args.add_argument('--build_type', help='CMAKE_BUILD_TYPE', required=True)
    args.add_argument('--stub_output', help='output directory for stubs', required=True)
    args = args.parse_args()
    config = build_utils.get_config(args.config, args.build_type)
    target_dir = build_utils.get_pyuipc_target_dir(args.binary_dir, config)
    print(f'Generating stubs for pyuipc from {target_dir}')
    success = generate_stubs(target_dir, args.stub_output)
    if not success:
        print(f'Error: failed to generate stubs')
        sys.exit(1)
    print('Stub generation completed successfully')

if __name__ == '__main__':
    main()

