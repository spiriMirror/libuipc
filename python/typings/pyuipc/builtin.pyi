aim_position: str
aim_transform: str
backend_abd_body_offset: str
backend_fem_vertex_offset: str
constitution_uid: str
constraint_uid: str
contact_element_id: str
extra_constitution_uids: str
gravity: str
implicit_geometry_uid: str
is_constrained: str
is_dynamic: str
is_facet: str
is_fixed: str
is_surf: str
mass_density: str
orient: str
parent_id: str
position: str
thickness: str
topo: str
transform: str
volume: str

class ConstitutionUIDCollection(UIDRegister):
    def __init__(self, *args, **kwargs) -> None: ...
    @staticmethod
    def instance() -> ConstitutionUIDCollection: ...

class ImplicitGeometryUIDCollection(UIDRegister):
    def __init__(self, *args, **kwargs) -> None: ...
    @staticmethod
    def instance() -> ImplicitGeometryUIDCollection: ...

class UIDRegister:
    def __init__(self, *args, **kwargs) -> None: ...
    def exists(self, arg0: int) -> bool: ...
    def find(self, *args, **kwargs): ...
    def to_json(self) -> json: ...
