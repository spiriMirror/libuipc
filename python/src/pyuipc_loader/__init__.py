import sys
import os 
import pathlib
import logging 

import subprocess
import ctypes
import json

this_file_dir = os.path.dirname(__file__)
if os.name == 'nt':
    sys.path.append(this_file_dir + '/Release/bin')
    sys.path.append(this_file_dir + '/RelWithDebInfo/bin')
elif os.name == 'posix':
    sys.path.append(this_file_dir + '/bin')
else:
    raise Exception("Unsupported OS")

import pyuipc

def init():
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent

    # generate stub
    logging.debug("Loading pyuipc", module_dir)

    config = pyuipc.default_config()
    config["module_dir"] = str(module_dir)
    pyuipc.init(config)

    logging.debug("pyuipc loaded: ", config)

init()