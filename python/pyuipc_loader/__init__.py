import sys
import os 
import json
import pathlib
import subprocess

with open(os.path.dirname(__file__) + '/../../output/build_info.json') as f:
    build_info = json.load(f)

sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/Release/bin')
sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/RelWithDebInfo/bin')

import pyuipc
def init():
    this_dir = pathlib.Path(__file__).absolute().parent
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent
    output_path = this_dir.parent / 'typings'
    print("Load pyuipc:", module_path)
    
    subprocess.run(['stubgen', 
                    '-p',  'pyuipc', 
                    "-o", output_path,
                    ], cwd=module_dir)

init()

