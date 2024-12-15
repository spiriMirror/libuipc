from .module_base import ModuleBase 
import os 

class PyUIPCModule(ModuleBase):
    def __init__(self):
        cwds = []
        cwd = os.path.dirname(os.path.abspath(__file__))
        if os.name == 'nt':
            cwds = [cwd + '/Release/bin', cwd + '/RelWithDebInfo/bin']
        elif os.name == 'posix':
            cwds = [cwd + '/Release/bin', cwd + '/RelWithDebInfo/bin']
        else:
            raise Exception('Unsupported OS')
        super().__init__("pyuipc", "pyuipc", cwds)

        if self._module.__file__ is None:
            err_message = '''Python binding is not built.
            Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYBIND=1` to enable python binding.'''
            raise Exception(err_message)
    
        # config = self._module.default_config()
        # config['module_dir'] = str(cwd)
        # self.init(config)

    def init(self, config):
        self.lazy_call_func("init")(config)

    def __getitem__(self, key: str):
        return self.lazy_get_attr(key)