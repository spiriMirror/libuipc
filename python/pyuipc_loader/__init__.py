import sys
import os 
import json
import pathlib

with open(os.path.dirname(__file__) + '/../../output/build_info.json') as f:
    build_info = json.load(f)

sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/Release/bin')
sys.path.append(build_info['CMAKE_BINARY_DIR'] + '/RelWithDebInfo/bin')

import pyuipc

pyuipc_path = pathlib.Path(pyuipc.__file__).absolute()
print("Load pyuipc:", pyuipc_path)