import os 
import sys
import shutil
import project_dir
import argparse as ap
import pathlib
from mypy import stubgen
import subprocess as sp

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

def copy_shared_libs(binary_dir, pyuipc_lib)->pathlib.Path:
    shared_lib_ext = ''
    if os.name == 'posix':
        shared_lib_ext = '.so'
    elif os.name == 'nt': # windows
        shared_lib_ext = '.dll'
    else:
        raise Exception('Unsupported OS')
    
    target_dir = binary_dir / 'python' / 'src' / 'pyuipc_loader' / config / 'bin'
    shared_lib_dir = binary_dir / config / 'bin'
    
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f'Create target directory {target_dir}')

    print(f'Copying shared library to {target_dir}:')
    # copy the pyuipc shared library to the target directory
    print(f'Copying {pyuipc_lib} to {target_dir}')
    shutil.copy(pyuipc_lib, target_dir)

    for file in os.listdir(shared_lib_dir):
        if file.endswith(shared_lib_ext):
            print(f'Copying {file}')
            full_path_file = shared_lib_dir / file
            shutil.copy(full_path_file, target_dir)
    
    return target_dir

def generate_stub(target_dir):
    typings_dir = binary_dir / 'python' / 'typings'
    
    print(f'Clear typings directory: {typings_dir}')
    shutil.rmtree(typings_dir, ignore_errors=True)

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
        packages=['pyuipc'],
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
    build_output_dir = binary_dir
    
    print(f'Copying package code to the target directory:')
    copy_python_source_code(proj_dir / 'python', binary_dir / 'python')
    
    config = get_config(args.config, args.build_type)
    
    print(f'Copying shared libraries to the target directory:')
    target_dir = copy_shared_libs(binary_dir, pyuipc_lib)
    
    print(f'Generating stubs:')
    generate_stub(target_dir)
    
    # flush the buffer to avoid the output being mixed
    sys.stdout.flush() 
    sys.stderr.flush()
    
    print(f'Installing the package to Python Environment: {sys.executable}')
    install_package(binary_dir)
