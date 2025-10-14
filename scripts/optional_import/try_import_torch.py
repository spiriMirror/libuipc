from .enabled_modules import EnabledModules
try:
    import torch
    EnabledModules.insert("torch")
except ImportError:
    pass