import os 
import shutil
import project_dir
import argparse as ap
import pathlib
import subprocess

args = ap.ArgumentParser(description='Copy the release directory to the project directory')
args.add_argument('--dir', help='library output directory', required=True)
args.add_argument('--config', help='configuration', required=True)
args = args.parse_args()

dir = args.dir
config = args.config

if config == 'Debug':
    raise Exception('Debug configuration is not supported, please use RelWithDebInfo or Release')

print(f'module dir: {dir}')
print(f'config: {config}')

source_dir = pathlib.Path(dir)
proj_dir = project_dir.project_dir()
target_dir = proj_dir / 'python' / 'src' / 'pyuipc_loader' / config / 'bin'

if not os.path.exists(target_dir):
    os.makedirs(target_dir)
    print(f'create target directory {target_dir}')

if os.name == 'posix': # linux
    shared_lib_ext = '.so'
elif os.name == 'nt': # windows
    shared_lib_ext = '.dll'
else:
    raise Exception('Unsupported OS')

print(f'copying shared library to {target_dir}')
for file in os.listdir(source_dir):
    if file.endswith(shared_lib_ext):
        print(f'Copying {file} to {target_dir}')
        full_path_file = f'{source_dir}/{file}'
        shutil.copy(full_path_file, target_dir)

typings_dir = proj_dir / 'python' / 'typings'
this_dir = pathlib.Path(__file__).absolute().parent
output_path = this_dir.parent.parent / 'typings'
print(f'Generating stubs to {typings_dir}, cwd={target_dir}')
R = subprocess.run(['stubgen', 
            '-p',  'pyuipc', 
            "-o", typings_dir,
            "-v"
            ], cwd=target_dir)

if R.returncode != 0:
    raise Exception("Failed to generate stubs, try `pip install mypy`")