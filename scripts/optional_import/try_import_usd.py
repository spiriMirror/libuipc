from .enabled_modules import EnabledModules
try:
    import pxr
    import pxr.Usd
    EnabledModules.insert("usd")
except ImportError:
    pass