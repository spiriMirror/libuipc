import sys
import os 
import pathlib
import logging 
import json

this_file_dir = os.path.dirname(__file__)
if os.name not in ['nt', 'posix']:
    raise Exception('Unsupported platform: ' + os.name)

sys.path.append(this_file_dir + '/modules/Release/bin')
sys.path.append(this_file_dir + '/modules/RelWithDebInfo/bin')
sys.path.append(this_file_dir + '/modules/releasedbg')
sys.path.append(this_file_dir + '/modules/release')

import pyuipc

def init():
    if pyuipc.__file__ is None:
        err_message = '''Python binding is not built.
        Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYBIND=1` to enable python binding.'''
        raise Exception(err_message)
    
    # get module path
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent

    # generate stub
    logging.debug('Loading pyuipc', module_dir)

    config = pyuipc.default_config()
    config['module_dir'] = str(module_dir)
    pyuipc.init(config)

    logging.debug('pyuipc loaded: ', config)

init()

from pyuipc import *
__version__ = pyuipc.__version__