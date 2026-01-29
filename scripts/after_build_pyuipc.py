import os 
import sys
import shutil
import project_dir
import argparse as ap
import pathlib
import subprocess as sp
import generate_stubs as gen_stubs

def flush_info():
    sys.stdout.flush()
    sys.stderr.flush()

def get_config(config: str, build_type: str):
    ret_config = ''
    
    if config != '' and build_type != '' and config != build_type:
        print(f'''Configuration and build type do not match, config={config}, build_type={build_type}.
This may be caused by the incorrect build command.
For Windows:
    cmake -S <source_dir>
    cmake --build <build_dir> --config <Release/RelWithDebInfo>
For Linux:
    cmake -S <source_dir> -DCMAKE_BUILD_TYPE=<Release/RelWithDebInfo>
    cmake --build <build_dir>
Ref: https://stackoverflow.com/questions/19024259/how-to-change-the-build-type-to-release-mode-in-cmake''')
        raise Exception('Configuration and build type do not match')

    if config == 'Debug' or build_type == 'Debug':
        raise Exception('Debug configuration is not supported, please use RelWithDebInfo or Release')
    
    if build_type == '' and config == '':
        ret_config = 'Release'
    else:
        ret_config = build_type if build_type != '' else config
    
    return ret_config

def copy_python_source_code(src_dir, bin_dir):
    paths = [
        'src/',
        'pyproject.toml',
        'setup.py'
    ]
    
    # copy the folders and files to the target directory
    for path in paths:
        src = src_dir / path
        dst = bin_dir / path
        print(f'Copying {src} to {dst}')
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            shutil.copy(src, dst)

def copy_shared_libs(binary_dir, pyuipc_lib, config)->pathlib.Path:
    shared_lib_exts = ['.so', '.dylib', '.dll']
    target_dir = binary_dir / 'python' / 'src' / 'uipc' / '_native' / config / 'bin'
    shared_lib_dir = binary_dir / config / 'bin'
    
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f'Create target directory {target_dir}')

    print(f'Copying shared library to {target_dir}:')
    # copy the pyuipc shared library to the target directory
    print(f'Copying {pyuipc_lib} to {target_dir}')
    shutil.copy(pyuipc_lib, target_dir)

    for file in os.listdir(shared_lib_dir):
        file = str(file)
        if file.endswith(tuple(shared_lib_exts)):
            print(f'Copying {file}')
            full_path_file = shared_lib_dir / file
            shutil.copy(full_path_file, target_dir)

    return target_dir

def generate_stubs(target_dir, stub_output):
    success = gen_stubs.generate_stubs(target_dir, stub_output)
    if not success:
        raise Exception('Failed to generate stubs')


def install_package(binary_dir):
    ret = sp.check_call([sys.executable, '-m', 'pip', 'install', f'{binary_dir}/python'])
    if ret != 0:
        print(f'''Automatically installing the package failed.
Please install the package manually by running:
{sys.executable} -m pip install {binary_dir}/python''')
        sys.exit(1)

if __name__ == '__main__':
    args = ap.ArgumentParser(description='Copy the release directory to the project directory')
    args.add_argument('--target', help='target pyuipc shared library', required=True)
    args.add_argument('--binary_dir', help='CMAKE_BINARY_DIR', required=True)
    args.add_argument('--config', help='$<CONFIG>', required=True)
    args.add_argument('--build_type', help='CMAKE_BUILD_TYPE', required=True)
    args = args.parse_args()
    
    print(f'config($<CONFIG>): {args.config} | build_type(CMAKE_BUILD_TYPE): {args.build_type}')
    
    pyuipc_lib = args.target
    binary_dir = pathlib.Path(args.binary_dir)
    proj_dir = pathlib.Path(project_dir.project_dir())
    
    print(f'Copying package code to the target directory:')
    copy_python_source_code(proj_dir / 'python', binary_dir / 'python')
    flush_info()
    
    config = get_config(args.config, args.build_type)
    
    print(f'Copying shared libraries to the target directory:')
    target_dir = copy_shared_libs(binary_dir, pyuipc_lib, config)
    flush_info()
    
    print(f'Generating stubs:')
    generate_stubs(target_dir, args.stub_output)
    flush_info()
