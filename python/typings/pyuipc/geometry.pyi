import numpy
import pyuipc
from typing import Iterator, overload

class AttributeCollection:
    def __init__(self, *args, **kwargs) -> None: ...
    def attribute_count(self) -> int: ...
    def clear(self) -> None: ...
    @overload
    def create(self, name: str, value: int) -> IAttributeSlot: ...
    @overload
    def create(self, name: str, value: int) -> IAttributeSlot: ...
    @overload
    def create(self, name: str, value: int) -> IAttributeSlot: ...
    @overload
    def create(self, name: str, value: float) -> IAttributeSlot: ...
    @overload
    def create(self, name: str, value: numpy.ndarray[numpy.float64]) -> IAttributeSlot: ...
    @overload
    def create(self, name: str, value: numpy.ndarray[numpy.int32]) -> IAttributeSlot: ...
    @overload
    def create(self, arg0: str, arg1: str) -> AttributeSlotString: ...
    def destroy(self, arg0: str) -> None: ...
    def find(self, arg0: str) -> IAttributeSlot: ...
    def reorder(self, arg0: numpy.ndarray[numpy.uint64]) -> None: ...
    def reserve(self, arg0: int) -> None: ...
    def resize(self, arg0: int) -> None: ...
    def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
    def size(self) -> int: ...

class AttributeIO:
    def __init__(self, file: str) -> None: ...
    def read(self, name: str, slot: IAttributeSlot) -> None: ...

class AttributeSlotFloat(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotI32(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.int32]: ...

class AttributeSlotI64(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.int64]: ...

class AttributeSlotMatrix12x12(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotMatrix2x2(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotMatrix3x3(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotMatrix4x4(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotMatrix6x6(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotMatrix9x9(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotString(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> CStringSpan: ...

class AttributeSlotU32(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.uint32]: ...

class AttributeSlotU64(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.uint64]: ...

class AttributeSlotVector12(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotVector2(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotVector2i(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.int32]: ...

class AttributeSlotVector3(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotVector3i(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.int32]: ...

class AttributeSlotVector4(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotVector4i(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.int32]: ...

class AttributeSlotVector6(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class AttributeSlotVector9(IAttributeSlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def view(self) -> numpy.ndarray[numpy.float64]: ...

class CStringSpan:
    def __init__(self) -> None: ...
    def __getitem__(self, arg0: int) -> str: ...
    def __iter__(self) -> Iterator[str]: ...
    def __len__(self) -> int: ...

class Geometry:
    class InstanceAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def clear(self) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def reserve(self, arg0: int) -> None: ...
        def resize(self, arg0: int) -> None: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def size(self) -> int: ...
        def to_json(self) -> json: ...

    class MetaAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def to_json(self) -> json: ...
    def __init__(self, *args, **kwargs) -> None: ...
    def instances(self) -> Geometry.InstanceAttributes: ...
    def meta(self) -> Geometry.MetaAttributes: ...
    def to_json(self) -> json: ...
    def type(self) -> str: ...

class GeometrySlot:
    def __init__(self, *args, **kwargs) -> None: ...
    def geometry(self) -> Geometry: ...
    def id(self) -> int: ...

class IAttributeSlot:
    def __init__(self, *args, **kwargs) -> None: ...
    def allow_destroy(self) -> bool: ...
    def is_shared(self) -> bool: ...
    def name(self) -> str: ...
    def size(self) -> int: ...
    def type_name(self) -> str: ...
    def view(self) -> numpy.ndarray: ...

class ImplicitGeometry(Geometry):
    def __init__(self) -> None: ...

class ImplicitGeometrySlot(GeometrySlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def geometry(self) -> ImplicitGeometry: ...

class SimplicialComplex(Geometry):
    class EdgeAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def clear(self) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def reserve(self, arg0: int) -> None: ...
        def resize(self, arg0: int) -> None: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def size(self) -> int: ...
        def to_json(self) -> json: ...
        def topo(self) -> AttributeSlotVector2i: ...

    class TetrahedronAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def clear(self) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def reserve(self, arg0: int) -> None: ...
        def resize(self, arg0: int) -> None: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def size(self) -> int: ...
        def to_json(self) -> json: ...
        def topo(self) -> AttributeSlotVector4i: ...

    class TriangleAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def clear(self) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def reserve(self, arg0: int) -> None: ...
        def resize(self, arg0: int) -> None: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def size(self) -> int: ...
        def to_json(self) -> json: ...
        def topo(self) -> AttributeSlotVector3i: ...

    class VertexAttributes:
        def __init__(self, *args, **kwargs) -> None: ...
        def clear(self) -> None: ...
        def create(self, arg0: str, arg1: object) -> IAttributeSlot: ...
        def destroy(self, arg0: str) -> None: ...
        def find(self, arg0: str) -> IAttributeSlot: ...
        def reserve(self, arg0: int) -> None: ...
        def resize(self, arg0: int) -> None: ...
        def share(self, arg0: str, arg1: IAttributeSlot) -> None: ...
        def size(self) -> int: ...
        def to_json(self) -> json: ...
    def __init__(self, *args, **kwargs) -> None: ...
    def copy(self) -> SimplicialComplex: ...
    def edges(self) -> SimplicialComplex.EdgeAttributes: ...
    def positions(self) -> AttributeSlotVector3: ...
    def tetrahedra(self) -> SimplicialComplex.TetrahedronAttributes: ...
    def transforms(self) -> AttributeSlotMatrix4x4: ...
    def triangles(self) -> SimplicialComplex.TriangleAttributes: ...
    def vertices(self) -> SimplicialComplex.VertexAttributes: ...

class SimplicialComplexIO:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, arg0: pyuipc.Transform) -> None: ...
    @overload
    def __init__(self, arg0: numpy.ndarray[numpy.float64]) -> None: ...
    def read(self, arg0: str) -> SimplicialComplex: ...
    def write(self, arg0: str, arg1: SimplicialComplex) -> None: ...

class SimplicialComplexSlot(GeometrySlot):
    def __init__(self, *args, **kwargs) -> None: ...
    def geometry(self) -> SimplicialComplex: ...

class SpreadSheetIO:
    def __init__(self, output_folder: str = ...) -> None: ...
    @overload
    def write_csv(self, geo_name: str, simplicial_complex: SimplicialComplex) -> None: ...
    @overload
    def write_csv(self, arg0: SimplicialComplex) -> None: ...
    @overload
    def write_json(self, geo_name: str, simplicial_complex: SimplicialComplex) -> None: ...
    @overload
    def write_json(self, arg0: SimplicialComplex) -> None: ...

class StringSpan:
    def __init__(self) -> None: ...
    def __getitem__(self, arg0: int) -> str: ...
    def __iter__(self) -> Iterator[str]: ...
    def __len__(self) -> int: ...
    def __setitem__(self, arg0: int, arg1: str) -> None: ...

def apply_region(arg0: SimplicialComplex) -> list: ...
def apply_transform(arg0: SimplicialComplex) -> list: ...
@overload
def extract_surface(arg0: SimplicialComplex) -> SimplicialComplex: ...
@overload
def extract_surface(arg0: list) -> SimplicialComplex: ...
def facet_closure(arg0: SimplicialComplex) -> SimplicialComplex: ...
def flip_inward_triangles(arg0: SimplicialComplex) -> SimplicialComplex: ...
def ground(height: float = ..., N: numpy.ndarray[numpy.float64] = ...) -> ImplicitGeometry: ...
def label_connected_vertices(arg0: SimplicialComplex) -> AttributeSlotI32: ...
def label_region(arg0: SimplicialComplex) -> None: ...
def label_surface(arg0: SimplicialComplex) -> None: ...
def label_triangle_orient(arg0: SimplicialComplex) -> AttributeSlotI32: ...
def linemesh(arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.int32]) -> SimplicialComplex: ...
def merge(arg0: list) -> SimplicialComplex: ...
@overload
def optimal_transform(arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.float64]) -> numpy.ndarray[numpy.float64]: ...
@overload
def optimal_transform(arg0: SimplicialComplex, arg1: SimplicialComplex) -> numpy.ndarray[numpy.float64]: ...
def pointcloud(arg0: numpy.ndarray[numpy.float64]) -> SimplicialComplex: ...
def tetmesh(arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.int32]) -> SimplicialComplex: ...
def tetrahedralize(simplicial_complex: SimplicialComplex, options: json = ...) -> SimplicialComplex: ...
def trimesh(arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.int32]) -> SimplicialComplex: ...
