__all__ = [
    'uipc',
    '__doc__',
    '__version__',
    'builtin',
    'Vector2',
    'Vector3',
    'Matrix3x3',
    'Matrix4x4',
    'Transform',
    'Quaternion',
    'AngleAxis',
    'Float',
    "Scene",
    "Engine",
    "World",
    "Animation",
    'SceneIO'
    'Logger',
    'Timer',
    "view"
]

from .module import PyUIPCModule
uipc = PyUIPCModule()
__doc__ = uipc["__doc__"]
__version__ = uipc["__version__"]

Vector2 = uipc["Vector2"]
Vector3 = uipc["Vector3"]
Matrix3x3 = uipc["Matrix3x3"]
Matrix4x4 = uipc["Matrix4x4"]
Transform = uipc["Transform"]
Quaternion = uipc["Quaternion"]
AngleAxis = uipc["AngleAxis"]
Float = uipc["Float"]
SceneIO = uipc["SceneIO"]
Logger = uipc["Logger"]
Timer = uipc["Timer"]
view = uipc["view"]

# special import
World = uipc["World"]
Engine = uipc["Engine"]
Scene = uipc["Scene"]
builtin = uipc["builtin"]
Animation = uipc["Animation"]