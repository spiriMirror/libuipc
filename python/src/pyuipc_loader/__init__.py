import sys
import os 
import pathlib
import logging 

import subprocess
import ctypes
import json

this_file_dir = os.path.dirname(__file__)
sys.path.append(this_file_dir + '/Release/bin')
sys.path.append(this_file_dir + '/RelWithDebInfo/bin')


import pyuipc 

def generate_stub(module_dir):
    this_dir = pathlib.Path(__file__).absolute().parent
    output_path = this_dir.parent.parent / 'typings'

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
    # generate stub
    logging.debug("Loading pyuipc", module_dir)

    config = pyuipc.default_config()
    config["module_dir"] = str(module_dir)
    pyuipc.init(config)

    logging.debug("pyuipc loaded: ", config)

init()

from pyuipc.engine import *
from pyuipc.world import *
