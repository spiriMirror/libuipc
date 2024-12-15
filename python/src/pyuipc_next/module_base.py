import sys 
import os 
import importlib 
import logging 

class ModuleBase:
    _name: str 
    _module_name: str 
    _cwd: str 
    _module = None

    def __init__(self, name: str, module_name: str, cwd: str):
        self._name = name
        self._module_name = module_name
        self._cwd = cwd
        logging.info("loading module {} from {}".format(self._module_name, self._cwd))
        self.load_module()

    def load_module(self):
        sys.path.append(self._cwd)
        try:
            self._module = importlib.import_module(self._module_name)
            logging.info("module {} loaded".format(self._module_name))
        except ImportError:
            logging.error("module {} not found".format(self._module_name))
            return None

    def lazy_get_attr(self, name: str):
        assert self._module is not None, "module is not loaded".format(self._module_name)
        return getattr(self._module, name)

    def lazy_call_func(self, name: str):
        assert self._module is not None, "module is not loaded".format(self._module_name)
        def _call(*args, **kwargs):
            logging.info("calling {} in module {}".format(name, self._module_name))
            return getattr(self._module, name)(*args, **kwargs)
        
        return _call 