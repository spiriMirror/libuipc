__all__ = [
    'uipc',
    '__doc__',
    '__version__',
    'geometry',
    'unit',
    "builtin",
    'Vector3',
    'Float',
    'Scene',
    'SceneIO'
]

from .module import PyUIPCModule
uipc = PyUIPCModule()
__doc__ = uipc["__doc__"]
__version__ = uipc["__version__"]
geometry = uipc["geometry"]
unit = uipc["unit"]
Vector3 = uipc["Vector3"]
Float = uipc["Float"]
Scene = uipc["Scene"]
SceneIO = uipc["SceneIO"]
builtin = uipc["builtin"]