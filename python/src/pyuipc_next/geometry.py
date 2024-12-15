from . import pyuipc 

class TetMesh:
    def __init__(self, Vs, Ts):
        self.Vs = Vs 
        self.Ts = Ts
        self.sc = pyuipc.geometry.tetmesh(Vs, Ts)
        self.attr_map = {}
    
    def resize(self, n: int):
        self.sc.instances().resize(n)

    def create_attr(self, name: str, dtype: str):
        self.attr_map[name] = self.sc.instances().create(name, dtype)
        return self.attr_map[name]
    
    def get_attr(self, name: str):
        return pyuipc.view(self.attr_map[name])

    # operator []
    def __getitem__(self, key):
        return self.get_attr(key)