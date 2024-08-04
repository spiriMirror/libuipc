import pyuipc.geometry
import pyuipc.world

class AffineBodyConstitution(pyuipc.world.IConstitution):
    def __init__(self, config: json = ...) -> None: ...
    def apply_to(self, sc: pyuipc.geometry.SimplicialComplex, kappa: float, mass_density: float = ...) -> None: ...
    @staticmethod
    def default_config() -> json: ...
