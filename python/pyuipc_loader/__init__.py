import sys
import os 
import json
import pathlib
import subprocess
import ctypes

class AssetDir:
    this_file = pathlib.Path(os.path.dirname(__file__)).resolve()
    _output_path = pathlib.Path(this_file / '../../output/').resolve()
    _assets_path = pathlib.Path(this_file / '../../assets/').resolve()
    _tetmesh_path = _assets_path / 'sim_data' / 'tetmesh'
    _trimesh_path = _assets_path / 'sim_data' / 'trimesh'
    
    def asset_path():
        return str(AssetDir._assets_path)

    def tetmesh_path():
        return str(AssetDir._tetmesh_path)

    def trimesh_path():
        return str(AssetDir._trimesh_path)
    
    def output_path(file):
        file_dir = pathlib.Path(file).absolute()
        this_python_root = AssetDir.this_file.parent.parent
        # get the relative path from the python root to the file
        relative_path = file_dir.relative_to(this_python_root)
        # construct the output path
        output_dir = AssetDir._output_path / relative_path / ''
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        return str(output_dir)
    
    def folder(file):
        return pathlib.Path(file).absolute().parent

with open(f'{AssetDir._output_path}/build_info.json') as f:
    build_info = json.load(f)

sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/Release/bin')
sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/RelWithDebInfo/bin')

import pyuipc

def generate_stub(module_dir):
    this_dir = pathlib.Path(__file__).absolute().parent
    output_path = this_dir.parent / 'typings'

    R = subprocess.run(['stubgen', 
                '-p',  'pyuipc', 
                "-o", output_path,
                ], cwd=module_dir)
    
    if R.returncode != 0:
        raise Exception("Failed to generate stubs, try `pip install mypy`")

def init():
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent
    
    generate_stub(module_dir)
    
    print("Load Pyuipc:", module_path)
    
    config = pyuipc.default_config()
    config['module_dir'] = str(module_dir)
    pyuipc.init(config)
    
    print("Init Pyuipc with config:", config)

init()


