import shutil
import sys
import pathlib as pl
import subprocess as sp
import pxr

def pxr_package_dir():
    return pl.Path(pxr.__file__).parent.resolve()

def this_folder():
    return pl.Path(__file__).parent.resolve()

def uipc_root_dir():
    this_file = pl.Path(__file__).resolve()
    # find the .uipc_root file upwards in the directory tree
    folders = []
    for parent in this_file.parents:
        folders.append(parent)
        if (parent / '.uipc_root').exists():
            return parent
    raise FileNotFoundError(f'Cannot find .uipc_root in any parent directory of {this_file}, searched: {folders}')

def gen_schema():
    folder = this_folder()
    schema_path = folder / 'schema.usda'
    cmd = [
        'usdGenSchema',
        str(schema_path),
        str(folder)
    ]
    print(f'Generating USD schema from {schema_path} to {folder}')
    sp.run(cmd, check=True)
    
    # substitute PATH in plugInfo
    plug_info_path = folder / 'plugInfo.json'
    with plug_info_path.open('r') as f:
        plug_info = f.read()
    plug_info = plug_info.replace('@PLUG_INFO_LIBRARY_PATH@', '.')
    plug_info = plug_info.replace('@PLUG_INFO_RESOURCE_PATH@', '.')
    plug_info = plug_info.replace('@PLUG_INFO_ROOT@', '.')
    with plug_info_path.open('w') as f:
        f.write(plug_info)

def remove_wrapper():
    # remove the generated file start with "wrap"
    folder = this_folder()
    for file in folder.iterdir():
        if file.name.startswith('wrap') and (file.suffix == '.h' or file.suffix == '.cpp'):
            print(f'Removing generated wrapper file {file}')
            file.unlink()

def move_headers():
    root = uipc_root_dir()
    dst = root / 'include' / 'uipc' / 'usd' / 'uipcPhysics'
    src = this_folder()
    header_files = list(src.glob('*.h'))
    for file in header_files:
        print(f'Moving {file} to {dst}')
        shutil.move(file, dst / file.name)

def main():
    gen_schema()
    remove_wrapper()
    move_headers()

if __name__ == '__main__':
    main()
