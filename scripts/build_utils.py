import pathlib
import os

def get_config(config: str, build_type: str)->str:
    '''
    Get the configuration for the build.
    Args:
        config: The configuration for the build.
        build_type: The build type for the build.
    Returns:
        The configuration for the build.
    Raises:
        Exception: If the configuration and build type do not match.
        Exception: If the debug configuration is not supported.
    '''
    
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

def get_pyuipc_target_dir(binary_dir: str, config: str)->str:
    target_dir = pathlib.Path(binary_dir) / 'python' / 'src' / 'uipc' / '_native' / config / 'bin'
    if not target_dir.exists():
        target_dir.mkdir(parents=True, exist_ok=True)
        print(f'Create target directory {str(target_dir)}')
    return str(target_dir)
