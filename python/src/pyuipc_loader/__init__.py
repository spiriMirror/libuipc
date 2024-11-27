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
    sys.path.append(this_file_dir + '/Release/bin')
    sys.path.append(this_file_dir + '/RelWithDebInfo/bin')
else:
    raise Exception("Unsupported OS")

import pyuipc

def init():
    if pyuipc is None:
        err_message = "Python binding is not built. \n           Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYBIND=1` to enable python binding."
        raise Exception(err_message)
    
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent

    # generate stub
    logging.debug("Loading pyuipc", module_dir)

    config = pyuipc.default_config()
    config["module_dir"] = str(module_dir)
    pyuipc.init(config)

    logging.debug("pyuipc loaded: ", config)

init()
