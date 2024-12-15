from .module_base import ModuleBase 
import os 

class PyUIPCModule(ModuleBase):
    def __init__(self):
        cwd = os.path.dirname(os.path.abspath(__file__)) + "/release"
        super().__init__("pyuipc", "pyuipc", cwd)

    @property
    def __doc__(self):
        return self.lazy_get_attr("__doc__")
    
    @property 
    def __version__(self):
        return self.lazy_get_attr("__version__")
    
    @property 
    def unit(self):
        return self.lazy_get_attr("unit")
    
    @property 
    def geometry(self):
        return self.lazy_get_attr("geometry")

    @property 
    def view(self):
        return self.lazy_get_attr("view")