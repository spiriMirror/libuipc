import sys
import os 
import pathlib
import json

this_file_dir = os.path.dirname(__file__)
# support windows, linux, macos
if sys.platform not in ['win32', 'linux', 'darwin']:
    raise Exception('Unsupported platform: ' + sys.platform)

sys.path.append(this_file_dir + '/_native/Release/bin')
sys.path.append(this_file_dir + '/_native/RelWithDebInfo/bin')
sys.path.append(this_file_dir + '/_native/releasedbg')
sys.path.append(this_file_dir + '/_native/release')

import pyuipc

def init():
    if pyuipc.__file__ is None:
        err_message = '''Python binding is not built.
        Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYBIND=1` to enable python binding.'''
        raise Exception(err_message)
    
    # get module path
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent

    config = pyuipc.default_config()
    config['module_dir'] = str(module_dir)
    pyuipc.init(config)

init()

from pyuipc import *
__version__ = pyuipc.__version__