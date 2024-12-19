from .module_base import ModuleBase 
import os 
import logging

class PyUIPCModule(ModuleBase):
    def __init__(self):
        cwds = []
        cwd = os.path.dirname(os.path.abspath(__file__))
        if os.name == 'nt':
            # cwds = [cwd + '/Release/bin', cwd + '/RelWithDebInfo/bin']
            cwds = [cwd + "/releasedbg"]
        elif os.name == 'posix':
            cwds = [cwd + '/Release/bin', cwd + '/RelWithDebInfo/bin']
        else:
            raise Exception('Unsupported OS')
        
        active_cwd = None
        for cwd in cwds:
            if os.path.exists(cwd):
                active_cwd = cwd
        
        if active_cwd is None:
            logging.error("module {} not found in {}".format(self._module_name, self._cwds))
            return None
    
        super().__init__("pyuipc", "pyuipc", active_cwd)

        if self._module.__file__ is None:
            err_message = '''Python binding is not built.
            Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYBIND=1` to enable python binding.'''
            raise Exception(err_message)
    
        config = self._module.default_config()
        config['module_dir'] = str(active_cwd)
        self.init(config)

    def init(self, config):
        self.lazy_call_func("init")(config)

    def __getitem__(self, key: str):
        return self.lazy_get_attr(key)