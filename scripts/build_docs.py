from project_dir import project_dir
import argparse as ap
import os
from pathlib import Path
import shutil
import subprocess as sp
import sys


def find_doxygen() -> Path | None:
    """Find Doxygen on PATH or in its standard Windows install location."""
    executable = shutil.which('doxygen')
    if executable:
        return Path(executable)

    if os.name == 'nt':
        for variable in ('ProgramFiles', 'ProgramFiles(x86)'):
            root = os.environ.get(variable)
            if not root:
                continue
            candidate = Path(root) / 'doxygen' / 'bin' / 'doxygen.exe'
            if candidate.is_file():
                return candidate

    return None


if __name__ == '__main__':
    proj_dir = project_dir()
    parser = ap.ArgumentParser(description='Build the documents')
    doc_default_dir = proj_dir.parent / 'libuipc-doc' / 'docs'
    parser.add_argument('-o', '--output', help='the output dir', default=f'{doc_default_dir}')
    args = parser.parse_args()
    doc_dir = args.output
    print(f'output_dir={doc_dir}')
    config_file = proj_dir / 'mkdocs-with-api.yaml'
    print(f'config_file={config_file}')
    doxygen = find_doxygen()
    if doxygen is None:
        print(
            'Doxygen was not found. Install Doxygen and add it to PATH before '
            'building the API documentation.',
            file=sys.stderr,
        )
        sys.exit(2)
    print(f'doxygen={doxygen}')
    env = os.environ.copy()
    env['PATH'] = f'{doxygen.parent}{os.pathsep}{env.get("PATH", "")}'
    result = sp.call(
        [
            sys.executable,
            '-m',
            'mkdocs',
            'build',
            '-f',
            str(config_file),
            '-d',
            str(doc_dir),
        ],
        cwd=proj_dir,
        env=env,
    )
    if result == 0:
        print('Success')
    else:
        print('Failure')
    sys.exit(result)
