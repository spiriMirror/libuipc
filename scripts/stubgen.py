import sys
import shutil
import argparse as ap
import pathlib
from mypy import stubgen

def generate_stub(source_dir, output_dir, build_type):
    PACKAGE_NAME = 'pyuipc'
    typings_dir = output_dir

    typings_folder = typings_dir / PACKAGE_NAME
    if typings_folder.exists():
        print(f'Clearing typings directory: {typings_folder}')
        shutil.rmtree(typings_folder)

    target_dir = source_dir / 'uipc' / 'modules' / build_type

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
        include_docstrings=False
    )

    try:
        stubgen.generate_stubs(options)
    except Exception as e:
        print(f'Error generating stubs: {e}')
        sys.exit(1)

if __name__ == '__main__':
    args = ap.ArgumentParser()
    args.add_argument('--source_dir', help='python source dir', required=True)
    args.add_argument('--output_dir', help='stub output dir', required=True)
    args.add_argument('--build_type', help='build system build type', required=True)
    args = args.parse_args()

    source_dir = pathlib.Path(args.source_dir)
    output_dir = pathlib.Path(args.output_dir)
    build_type = pathlib.Path(args.build_type)
    generate_stub(source_dir, output_dir, build_type)
