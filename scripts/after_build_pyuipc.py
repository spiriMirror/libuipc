import os 
import sys
import shutil
import project_dir
import argparse as ap
import pathlib
from mypy import stubgen

if __name__ == '__main__':
    args = ap.ArgumentParser(description='Copy the release directory to the project directory')
    args.add_argument('--target', help='target pyuipc shared library', required=True)
    args.add_argument('--binary_dir', help='cmake binary dir', required=True)
    args.add_argument('--config', help='$<CONFIG>', required=True)
    args.add_argument('--build_type', help='CMAKE_BUILD_TYPE', required=True)
    args = args.parse_args()

    pyuipc_lib = args.target
    binary_dir = args.binary_dir
    config = args.config
    build_type = args.build_type
    proj_dir = project_dir.project_dir()

    if config != '' and build_type != '' and config != build_type:
        raise Exception(f'Configuration and build type do not match, config={config}, build_type={build_type}')

    if config == 'Debug' or build_type == 'Debug':
        raise Exception('Debug configuration is not supported, please use RelWithDebInfo or Release')
    
    if build_type == '' and config == '':
        build_type = 'Release'
    
    cmake_binary_dir = binary_dir
    build_output_dir = pathlib.Path(binary_dir)

    if os.name == 'posix': # linux
        target_dir = proj_dir / 'python' / 'src' / 'pyuipc_loader' / build_type / 'bin'
        shared_lib_ext = '.so'
        build_output_dir = build_output_dir / build_type / 'bin'
    elif os.name == 'nt': # windows
        target_dir = proj_dir / 'python' / 'src' / 'pyuipc_loader' / config / 'bin'
        shared_lib_ext = '.dll'
        build_output_dir = build_output_dir / config / 'bin'
    else:
        raise Exception('Unsupported OS')

    source_dir = pathlib.Path(build_output_dir)
    print(f'pyuipc: {pyuipc_lib}')
    print(f'module dir: {build_output_dir}')
    print(f'config($<CONFIG>): {config} | build_type(CMAKE_BUILD_TYPE): {build_type}')


    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f'Create target directory {target_dir}')

    print(f'Copying shared library to {target_dir}:')
    # copy the pyuipc shared library to the target directory
    print(f'Copying {pyuipc_lib} to {target_dir}')
    shutil.copy(pyuipc_lib, target_dir)

    # recusively find all shared libraries and print them
    # for root, dirs, files in os.walk(source_dir):
    #     for file in files:
    #         if file.endswith(shared_lib_ext):
    #             print(f'Found shared library: {file}')

    for file in os.listdir(source_dir):
        if file.endswith(shared_lib_ext):
            print(f'Copying {file}')
            full_path_file = f'{source_dir}/{file}'
            shutil.copy(full_path_file, target_dir)

    typings_dir = proj_dir / 'python' / 'typings'
    this_dir = pathlib.Path(__file__).absolute().parent
    output_path = this_dir.parent.parent / 'typings'

    print(f'Try generating stubs to {typings_dir}')

    # add the target directory to the python path
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