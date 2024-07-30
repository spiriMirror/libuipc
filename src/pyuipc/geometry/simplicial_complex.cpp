#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/pyuipc.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_friend.h>

namespace uipc::geometry
{
template <>
class AttributeFriend<pyuipc::geometry::PySimplicialComplex>
{
  public:
    static S<IAttributeSlot> find(SimplicialComplex::VertexAttributes& v, std::string_view name)
    {
        return v.m_attributes.find(name);
    }

    static S<IAttributeSlot> find(SimplicialComplex::EdgeAttributes& e, std::string_view name)
    {
        return e.m_attributes.find(name);
    }

    static S<IAttributeSlot> find(SimplicialComplex::TriangleAttributes& t, std::string_view name)
    {
        return t.m_attributes.find(name);
    }

    static S<IAttributeSlot> find(SimplicialComplex::TetrahedronAttributes& t,
                                  std::string_view name)
    {
        return t.m_attributes.find(name);
    }
};
}  // namespace uipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

using Accessor = AttributeFriend<PySimplicialComplex>;

PySimplicialComplex::PySimplicialComplex(py::module& m)
{
    auto class_SimplicialComplex =
        py::class_<SimplicialComplex, Geometry>(m, "SimplicialComplex");

    class_SimplicialComplex.def(py::init<>());

    class_SimplicialComplex.def(
        "vertices", [](SimplicialComplex& self) { return self.vertices(); });

    class_SimplicialComplex.def(
        "positions",
        [&](SimplicialComplex& self) -> AttributeSlot<Vector3>&
        { return self.positions(); },
        py::return_value_policy::reference_internal);

    class_SimplicialComplex.def(
        "edges", [](SimplicialComplex& self) { return self.edges(); });

    class_SimplicialComplex.def(
        "triangles", [](SimplicialComplex& self) { return self.triangles(); });

    class_SimplicialComplex.def(
        "tetrahedra", [](SimplicialComplex& self) { return self.tetrahedra(); });


    {
        auto class_VertexAttributes =
            py::class_<SimplicialComplex::VertexAttributes>(class_SimplicialComplex,
                                                            "VertexAttributes");
        class_VertexAttributes.def(
            "find",
            [](SimplicialComplex::VertexAttributes& self, std::string_view name) -> IAttributeSlot&
            { return *Accessor::find(self, name); },
            py::return_value_policy::reference_internal);

        class_VertexAttributes.def("topo",
                                   [](SimplicialComplex::VertexAttributes& self)
                                   { return self.topo(); });

        auto class_SimplicalComplexVertexTopo =
            py::class_<SimplicialComplexTopo<VertexSlot>>(class_VertexAttributes, "Topo");
    }


    {
        auto class_EdgeAttributes =
            py::class_<SimplicialComplex::EdgeAttributes>(class_SimplicialComplex,
                                                          "EdgeAttributes");

        class_EdgeAttributes.def(
            "find",
            [](SimplicialComplex::EdgeAttributes& self, std::string_view name) -> IAttributeSlot&
            { return *Accessor::find(self, name); },
            py::return_value_policy::reference_internal);

        class_EdgeAttributes.def("topo",
                                 [](SimplicialComplex::EdgeAttributes& self)
                                 { return self.topo(); });

        auto class_SimplicalComplexEdgeTopo =
            py::class_<SimplicialComplexTopo<EdgeSlot>>(class_EdgeAttributes, "Topo");
    }

    {
        auto class_TriangleAttributes =
            py::class_<SimplicialComplex::TriangleAttributes>(class_SimplicialComplex,
                                                              "TriangleAttributes");
        class_TriangleAttributes.def(
            "find",
            [](SimplicialComplex::TriangleAttributes& self, std::string_view name) -> IAttributeSlot&
            { return *Accessor::find(self, name); },
            py::return_value_policy::reference_internal);


        class_TriangleAttributes.def("topo",
                                     [](SimplicialComplex::TriangleAttributes& self)
                                     { return self.topo(); });


        auto class_SimplicalComplexTriangleTopo =
            py::class_<SimplicialComplexTopo<TriangleSlot>>(class_TriangleAttributes, "Topo");
    }


    {
        auto class_TetrahedronAttributes =
            py::class_<SimplicialComplex::TetrahedronAttributes>(
                class_SimplicialComplex, "TetrahedronAttributes");
        class_TetrahedronAttributes.def(
            "find",
            [](SimplicialComplex::TetrahedronAttributes& self, std::string_view name) -> IAttributeSlot&
            { return *Accessor::find(self, name); },
            py::return_value_policy::reference_internal);

        class_TetrahedronAttributes.def("topo",
                                        [](SimplicialComplex::TetrahedronAttributes& self)
                                        { return self.topo(); });

        auto class_SimplicalComplexTetrahedronTopo =
            py::class_<SimplicialComplexTopo<TetrahedronSlot>>(class_TetrahedronAttributes,
                                                               "Topo");
    }
}
}  // namespace pyuipc::geometry
