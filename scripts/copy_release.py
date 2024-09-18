import os 
import shutil
import project_dir
import argparse as ap


args = ap.ArgumentParser(description='Copy the release directory to the project directory')
args.add_argument('--dir', help='library output directory', required=True)
args.add_argument('--config', help='configuration', required=True)
args = args.parse_args()

dir = args.dir
config = args.config

print(f'module dir: {dir}')
print(f'config: {config}')

source_dir = dir
proj_dir = project_dir.project_dir()
target_dir = f'{proj_dir}/python/pyuipc_loader/{config}/bin/'

if not os.path.exists(target_dir):
    os.makedirs(target_dir)
    print(f'create target directory {target_dir}')

# if linux
if os.name == 'posix':
    shared_lib_ext = '.so'
elif os.name == 'nt':
    shared_lib_ext = '.dll'
else:
    raise Exception('Unsupported OS')

# copy the shared libraries

for file in os.listdir(source_dir):
    if file.endswith(shared_lib_ext):
        print(f'Copying {file} to {target_dir}')
        full_path_file = f'{source_dir}/{file}'
        shutil.copy(full_path_file, target_dir)