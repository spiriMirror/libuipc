#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/pyuipc.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_friend.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>
#include <pyuipc/geometry/attribute_creator.h>
namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::geometry::PySimplicialComplex>
{
  public:
    template <IndexT N>
    static S<IAttributeSlot> find(SimplicialComplexAttributes<false, N>& a, std::string_view name)
    {
        return a.m_attributes.find(name);
    }

    template <IndexT N>
    static void share(SimplicialComplexAttributes<false, N>& a,
                      std::string_view                       name,
                      IAttributeSlot&                        b)
    {
        a.m_attributes.share(name, b);
    }

    template <IndexT N>
    static S<IAttributeSlot> create(SimplicialComplexAttributes<false, N>& a,
                                    std::string_view                       name,
                                    py::object object)
    {
        return pyuipc::geometry::AttributeCreator::create(a.m_attributes, name, object);
    }
};
}  // namespace uipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

using Accessor = AttributeFriend<PySimplicialComplex>;

template <IndexT N>
void def_method(py::module& m, py::class_<SimplicialComplexAttributes<false, N>>& class_Attribute)
{
    using Attributes = SimplicialComplexAttributes<false, N>;
    using TopoValueT = typename Attributes::TopoValueT;

    class_Attribute.def("find",
                        [](Attributes& self, std::string_view name)
                        { return Accessor::template find<N>(self, name); },
                        py::arg("name"),
                        R"(Find an attribute by name.
Args:
    name: Attribute name.
Returns:
    AttributeSlot or None: Attribute slot if found, None otherwise.)");

    if constexpr(N > 0)
    {
        class_Attribute.def(
            "topo",
            [](Attributes& self) -> AttributeSlot<TopoValueT>&
            { return self.topo(); },
            py::return_value_policy::reference_internal,
            R"(Get the topology attribute slot.
Returns:
    AttributeSlot: Reference to topology attribute slot.)");
    }

    class_Attribute.def("resize", &Attributes::resize,
                       py::arg("size"),
                       R"(Resize the attributes to the specified size.
Args:
    size: New size.)");
    class_Attribute.def("size", &Attributes::size,
                       R"(Get the number of simplices.
Returns:
    int: Number of simplices.)");
    class_Attribute.def("reserve", &Attributes::reserve,
                       py::arg("size"),
                       R"(Reserve capacity for attributes.
Args:
    size: Capacity to reserve.)");
    class_Attribute.def("clear", &Attributes::clear,
                       R"(Clear all attributes.)");
    class_Attribute.def("destroy", &Attributes::destroy,
                       py::arg("name"),
                       R"(Destroy an attribute by name.
Args:
    name: Attribute name to destroy.)");
    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
                        { Accessor::template share<N>(self, name, attribute); },
                        py::arg("name"),
                        py::arg("attribute"),
                        R"(Share an existing attribute slot with a new name.
Args:
    name: New name for the shared attribute.
    attribute: Attribute slot to share.)");

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object) {
                            return Accessor::template create<N>(self, name, object);
                        },
                        py::arg("name"),
                        py::arg("object"),
                        R"(Create a new attribute from a Python object.
Args:
    name: Attribute name.
    object: Python object to create attribute from.
Returns:
    AttributeSlot: Created attribute slot.)");

    class_Attribute.def("to_json", &Attributes::to_json,
                        R"(Convert attributes to JSON representation.
Returns:
    dict: JSON representation of the attributes.)");
}

PySimplicialComplex::PySimplicialComplex(py::module& m)
{
    // Class Def

    auto class_SimplicialComplex =
        py::class_<SimplicialComplex, Geometry, S<SimplicialComplex>>(m, "SimplicialComplex",
                                                                       R"(SimplicialComplex class representing a simplicial complex (mesh) with vertices, edges, triangles, and tetrahedra.)");


    auto class_VertexAttributes =
        py::class_<SimplicialComplex::VertexAttributes>(class_SimplicialComplex,
                                                        "VertexAttributes",
                                                        R"(Vertex attributes for the simplicial complex.)");

    auto class_EdgeAttributes =
        py::class_<SimplicialComplex::EdgeAttributes>(class_SimplicialComplex, "EdgeAttributes",
                                                       R"(Edge attributes for the simplicial complex.)");

    auto class_TriangleAttributes =
        py::class_<SimplicialComplex::TriangleAttributes>(class_SimplicialComplex,
                                                          "TriangleAttributes",
                                                          R"(Triangle attributes for the simplicial complex.)");

    auto class_TetrahedronAttributes =
        py::class_<SimplicialComplex::TetrahedronAttributes>(class_SimplicialComplex,
                                                             "TetrahedronAttributes",
                                                             R"(Tetrahedron attributes for the simplicial complex.)");

    // Method Def

    // NOTE: Don't allow python frontend to construct a SimplicialComplex directly
    // class_SimplicialComplex.def(py::init<>()); // removed

    class_SimplicialComplex.def(
        "transforms",
        [](SimplicialComplex& self) -> AttributeSlot<Matrix4x4>&
        { return self.transforms(); },
        py::return_value_policy::reference_internal,
        R"(Get the transform attribute slot (4x4 transformation matrices).
Returns:
    AttributeSlot: Reference to transform attribute slot.)");

    class_SimplicialComplex.def(
        "vertices",
        [](SimplicialComplex& self) { return self.vertices(); },
        py::return_value_policy::move,
        R"(Get the vertex attributes.
Returns:
    VertexAttributes: Vertex attributes collection.)");

    class_SimplicialComplex.def(
        "positions",
        [&](SimplicialComplex& self) -> AttributeSlot<Vector3>&
        { return self.positions(); },
        py::return_value_policy::reference_internal,
        R"(Get the position attribute slot (3D vertex positions).
Returns:
    AttributeSlot: Reference to position attribute slot.)");

    class_SimplicialComplex.def(
        "edges", [](SimplicialComplex& self) { return self.edges(); }, py::return_value_policy::move,
        R"(Get the edge attributes.
Returns:
    EdgeAttributes: Edge attributes collection.)");

    class_SimplicialComplex.def(
        "triangles",
        [](SimplicialComplex& self) { return self.triangles(); },
        py::return_value_policy::move,
        R"(Get the triangle attributes.
Returns:
    TriangleAttributes: Triangle attributes collection.)");

    class_SimplicialComplex.def(
        "tetrahedra",
        [](SimplicialComplex& self) { return self.tetrahedra(); },
        py::return_value_policy::move,
        R"(Get the tetrahedron attributes.
Returns:
    TetrahedronAttributes: Tetrahedron attributes collection.)");

    class_SimplicialComplex.def("copy",
                                [](SimplicialComplex& self) -> SimplicialComplex
                                { return self; },
                                R"(Create a copy of the simplicial complex.
Returns:
    SimplicialComplex: Copy of the simplicial complex.)");

    class_SimplicialComplex.def("dim", &SimplicialComplex::dim,
                               R"(Get the dimension of the simplicial complex.
Returns:
    int: Dimension (0=points, 1=edges, 2=triangles, 3=tetrahedra).)");

    class_SimplicialComplex.def("__repr__",
                                [](const SimplicialComplex& self)
                                { return fmt::format("{}", self); },
                                R"(String representation of the simplicial complex.
Returns:
    str: String representation.)");

    def_method(m, class_VertexAttributes);
    def_method(m, class_EdgeAttributes);
    def_method(m, class_TriangleAttributes);
    def_method(m, class_TetrahedronAttributes);
}
}  // namespace pyuipc::geometry
