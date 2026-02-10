import pathlib
from ._native import pyuipc

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

# import all pyuipc modules
from ._native.pyuipc import *
__version__ = pyuipc.__version__