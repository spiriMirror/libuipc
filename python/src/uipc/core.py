from . import uipc 

__all__ = [
    "Engine",
    "World",
    "Scene",
    "SceneIO",
    "Object",
    "Animation",
    "ContactElement",
]

core = uipc["core"]
Engine = core.Engine
World = core.World
Scene = core.Scene
SceneIO = core.SceneIO
Object = core.Object
Animation = core.Animation
ContactElement = core.ContactElement