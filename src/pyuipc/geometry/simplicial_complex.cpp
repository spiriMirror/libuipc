#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/pyuipc.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_friend.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>
namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::geometry::PySimplicialComplex>
{
  public:
    template <typename SlotT>
    static S<IAttributeSlot> find(SimplicialComplexAttributes<SlotT>& a, std::string_view name)
    {
        return a.m_attributes.find(name);
    }

    template <typename SlotT>
    static void share(SimplicialComplexAttributes<SlotT>& a, std::string_view name, IAttributeSlot& b)
    {
        a.m_attributes.share(name, b);
    }

    template <typename SlotT>
    static S<IAttributeSlot> create(SimplicialComplexAttributes<SlotT>& a,
                                    std::string_view                    name,
                                    py::object                          object)
    {
        auto pyobj = py::cast(a.m_attributes,
                              py::return_value_policy::reference_internal,  // member object is a reference in the parent object
                              py::cast(a)  // parent object
        );

        // call the create method of the member object
        return py::cast<S<IAttributeSlot>>(pyobj.attr("create").call(py::cast(name), object));
    }
};
}  // namespace uipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

using Accessor = AttributeFriend<PySimplicialComplex>;

template <typename SlotT>
void def_method(py::module& m, py::class_<SimplicialComplexAttributes<SlotT>>& class_Attribute)
{
    using Attributes = SimplicialComplexAttributes<SlotT>;

    class_Attribute.def("find",
                        [](Attributes& self, std::string_view name)
                        { return Accessor::template find<SlotT>(self, name); });

    class_Attribute.def(
        "topo", [](Attributes& self) { return self.topo(); }, py::return_value_policy::move);

    class_Attribute.def("view",
                        [](SimplicialComplexTopo<SlotT>& self) {
                            return as_numpy(std::move(self).view(), py::cast(self));
                        });

    m.def("view",
          [](SimplicialComplexTopo<SlotT>& self)
          { return as_numpy(view(std::move(self)), py::cast(self)); });

    class_Attribute.def("share",
                        [](SimplicialComplexTopo<SlotT>& self,
                           SimplicialComplexTopo<SlotT>& other)
                        { return std::move(self).share(std::move(other)); });

    class_Attribute.def("is_shared",
                        [](SimplicialComplexTopo<SlotT>& self)
                        { return std::move(self).is_shared(); });

    class_Attribute.def("resize", &Attributes::resize);
    class_Attribute.def("size", &Attributes::size);
    class_Attribute.def("reserve", &Attributes::reserve);
    class_Attribute.def("clear", &Attributes::clear);
    class_Attribute.def("destroy", &Attributes::destroy);
    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, IAttributeSlot& attribute) {
                            Accessor::template share<SlotT>(self, name, attribute);
                        });

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object) {
                            return Accessor::template create<SlotT>(self, name, object);
                        });

    class_Attribute.def("to_json", &Attributes::to_json);
}

template <typename SlotT>
void def_method(py::module& m, py::class_<SimplicialComplexTopo<SlotT>>& class_Attribute)
{
    class_Attribute.def("view",
                        [](SimplicialComplexTopo<SlotT>& self) {
                            return as_numpy(std::move(self).view(), py::cast(self));
                        });

    m.def("view",
          [](SimplicialComplexTopo<SlotT>& self)
          { return as_numpy(view(std::move(self)), py::cast(self)); });

    class_Attribute.def("share",
                        [](SimplicialComplexTopo<SlotT>& self,
                           SimplicialComplexTopo<SlotT>& other)
                        { return std::move(self).share(std::move(other)); });

    class_Attribute.def("is_shared",
                        [](SimplicialComplexTopo<SlotT>& self)
                        { return std::move(self).is_shared(); });
}


PySimplicialComplex::PySimplicialComplex(py::module& m)
{
    // Class Def

    auto class_SimplicialComplex =
        py::class_<SimplicialComplex, Geometry, S<SimplicialComplex>>(m, "SimplicialComplex");


    auto class_VertexAttributes =
        py::class_<SimplicialComplex::VertexAttributes>(class_SimplicialComplex,
                                                        "VertexAttributes");

    auto class_SimplicalComplexVertexTopo =
        py::class_<SimplicialComplexTopo<VertexSlot>>(class_VertexAttributes, "Topo");

    auto class_EdgeAttributes =
        py::class_<SimplicialComplex::EdgeAttributes>(class_SimplicialComplex, "EdgeAttributes");

    auto class_SimplicalComplexEdgeTopo =
        py::class_<SimplicialComplexTopo<EdgeSlot>>(class_EdgeAttributes, "Topo");

    auto class_TriangleAttributes =
        py::class_<SimplicialComplex::TriangleAttributes>(class_SimplicialComplex,
                                                          "TriangleAttributes");

    auto class_SimplicalComplexTriangleTopo =
        py::class_<SimplicialComplexTopo<TriangleSlot>>(class_TriangleAttributes, "Topo");

    auto class_TetrahedronAttributes =
        py::class_<SimplicialComplex::TetrahedronAttributes>(class_SimplicialComplex,
                                                             "TetrahedronAttributes");

    auto class_SimplicalComplexTetrahedronTopo =
        py::class_<SimplicialComplexTopo<TetrahedronSlot>>(class_TetrahedronAttributes, "Topo");

    // Method Def

    class_SimplicialComplex.def(py::init<>());

    class_SimplicialComplex.def(
        "vertices",
        [](SimplicialComplex& self) { return self.vertices(); },
        py::return_value_policy::move);

    class_SimplicialComplex.def(
        "positions",
        [&](SimplicialComplex& self) -> AttributeSlot<Vector3>&
        { return self.positions(); },
        py::return_value_policy::reference_internal);

    class_SimplicialComplex.def(
        "edges", [](SimplicialComplex& self) { return self.edges(); }, py::return_value_policy::move);

    class_SimplicialComplex.def(
        "triangles",
        [](SimplicialComplex& self) { return self.triangles(); },
        py::return_value_policy::move);

    class_SimplicialComplex.def(
        "tetrahedra",
        [](SimplicialComplex& self) { return self.tetrahedra(); },
        py::return_value_policy::move);


    class_VertexAttributes.def("find",
                               [](SimplicialComplex::VertexAttributes& self, std::string_view name)
                               { return Accessor::find(self, name); });

    // Vertex
    {
        def_method(m, class_VertexAttributes);

        def_method(m, class_SimplicalComplexVertexTopo);
    }

    // Edges
    {
        def_method(m, class_EdgeAttributes);

        def_method(m, class_SimplicalComplexEdgeTopo);
    }

    // Triangles
    {
        def_method(m, class_TriangleAttributes);

        def_method(m, class_SimplicalComplexTriangleTopo);
    }

    // Tetrahedra
    {
        def_method(m, class_TetrahedronAttributes);

        def_method(m, class_SimplicalComplexTetrahedronTopo);
    }

    class_SimplicialComplex.def("copy",
                                [](SimplicialComplex& self) -> SimplicialComplex
                                { return self; });
}
}  // namespace pyuipc::geometry
