import os 
import sys
import shutil
import argparse as ap
import pathlib
import subprocess as sp
import optional_import # help stubgen to detect optional modules' api

def is_option_on(option: str):
    # convert the option to uppercase
    option = option.upper()
    return option == 'ON' or option == 'TRUE' or option == '1' or option == 'YES' or option == 'Y'

def flush_info():
    sys.stdout.flush()
    sys.stderr.flush()

def shared_lib_pattern():
    if sys.platform == 'win32':
        return '*.dll'
    elif sys.platform == 'darwin':
        return '*.dylib'
    elif sys.platform == 'linux':
        return '*.so'
    else:
        raise Exception(f'Unsupported platform: {sys.platform}')

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

def clear_binary_python_dir(binary_dir):
    binary_python_dir = binary_dir / 'python'
    if os.path.exists(binary_python_dir):
        print(f'Clearing binary python directory: {binary_python_dir}')
        shutil.rmtree(binary_python_dir, ignore_errors=True)
    os.makedirs(binary_python_dir, exist_ok=True)

def copy_python_source_code(proj_dir, bin_dir):
    paths = [
        'src/',
        'pyproject.toml',
    ]
    src_dir = proj_dir / 'python'
    dst_dir = bin_dir / 'python'
    # copy the folders and files to the target directory
    for path in paths:
        src = src_dir / path
        dst = dst_dir / path
        print(f'Copying {src} to {dst}')
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            shutil.copy(src, dst)

def copy_shared_libs(config:str, binary_dir:pathlib.Path, pyuipc_lib:pathlib.Path)->pathlib.Path:
    shared_lib_dir = binary_dir / config / 'bin' # src
    target_dir = binary_dir / 'python' / 'src' / 'uipc' / '_native' # dst
    
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f'Create target directory {target_dir}')

    print(f'Target directory: {target_dir}')

    # copy the pyuipc shared library to the target directory
    print(f'Copying {pyuipc_lib}')
    shutil.copy(str(pyuipc_lib), str(target_dir))

    # copy dependent shared libraries to the target directory
    for file in shared_lib_dir.rglob(shared_lib_pattern()):
        print(f'Copying {file}')
        shutil.copy(file, target_dir)

    return target_dir

def generate_uipc_stubs(binary_dir):
    optional_import.EnabledModules.report()
    PACKAGE_NAME = 'uipc'

    typings_dir = binary_dir / 'python' / 'src'
    
    # clear the .pyi files in the typings directory
    typings_folder = typings_dir / PACKAGE_NAME
    for file in typings_folder.rglob('*.pyi'):
        print(f'Clear {file}')
        os.remove(file)

    # generate the stubs using nanobind's built-in stubgen
    print(f'Try generating stubs to {typings_dir}')
    sys.path.append(str(typings_dir))
    
    flush_info()
    
    args = [sys.executable, '-m', 'nanobind.stubgen',
            '-m', PACKAGE_NAME,
            '-o', str(typings_dir)]

    try:
        ret = sp.run(args, capture_output=False, text=True)
        if ret.returncode != 0:
            print(f'Error generating stubs: return code {ret.returncode}')
            sys.exit(1)
    except Exception as e:
        print(f'Error generating stubs: {e}')
        sys.exit(1)

def uninstall_package():
    # check if the package is installed
    ret = sp.run([sys.executable, '-m', 'pip', 'show', 'pyuipc'], capture_output=True, text=True)
    if ret.returncode == 0:
        print(f'Uninstalling the old package:')
        ret = sp.check_call([sys.executable, '-m', 'pip', 'uninstall', '-y', 'pyuipc'])
        if ret != 0:
            print(f'Error uninstalling package: {ret}')
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
    args.add_argument('--project_dir', help='project directory', required=True)
    args.add_argument('--binary_dir', help='CMAKE_BINARY_DIR', required=True)
    args.add_argument('--config', help='$<CONFIG>', required=True)
    args.add_argument('--build_type', help='CMAKE_BUILD_TYPE', required=True)
    args.add_argument('--build_wheel', help='UIPC_BUILD_PYTHON_WHEEL', required=True)
    args = args.parse_args()

    print(f'config($<CONFIG>): {args.config} | build_type(CMAKE_BUILD_TYPE): {args.build_type}')

    pyuipc_lib = pathlib.Path(args.target)
    binary_dir = pathlib.Path(args.binary_dir)
    proj_dir = pathlib.Path(args.project_dir)

    # clean up the old package, avoid polluting the stub generation
    print(f'Cleaning up the old package:')
    uninstall_package()

    print(f'Clearing binary python directory: {binary_dir}')
    clear_binary_python_dir(binary_dir)
    flush_info()

    print(f'Copying package code to the target directory:')
    copy_python_source_code(proj_dir, binary_dir)
    flush_info()

    print(f'Copying shared libraries to the target directory:')
    config = get_config(args.config, args.build_type)
    target_dir = copy_shared_libs(config, binary_dir, pyuipc_lib)
    flush_info()

    print(f'Generating stubs:')
    generate_uipc_stubs(binary_dir)
    flush_info()
    
    if not is_option_on(args.build_wheel):
        print(f'Installing the package to Python Environment: {sys.executable}')
        install_package(binary_dir)
        flush_info()