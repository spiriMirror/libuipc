from .module import PyUIPCModule

pyuipc = PyUIPCModule()
__doc__ = pyuipc.__doc__
__version__ = pyuipc.__version__
unit = pyuipc.unit
geometry = pyuipc.geometry

__all__ = [
    "__doc__", 
    "__version__",
    "pyuipc",
    "unit",
    "geometry"
]