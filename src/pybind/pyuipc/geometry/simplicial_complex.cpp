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
                        { return Accessor::template find<N>(self, name); });

    if constexpr(N > 0)
    {
        class_Attribute.def(
            "topo",
            [](Attributes& self) -> AttributeSlot<TopoValueT>&
            { return self.topo(); },
            py::return_value_policy::reference_internal);
    }

    class_Attribute.def("resize", &Attributes::resize);
    class_Attribute.def("size", &Attributes::size);
    class_Attribute.def("reserve", &Attributes::reserve);
    class_Attribute.def("clear", &Attributes::clear);
    class_Attribute.def("destroy", &Attributes::destroy);
    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
                        { Accessor::template share<N>(self, name, attribute); });

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object) {
                            return Accessor::template create<N>(self, name, object);
                        });

    class_Attribute.def("to_json", &Attributes::to_json);
}

PySimplicialComplex::PySimplicialComplex(py::module& m)
{
    // Class Def

    auto class_SimplicialComplex =
        py::class_<SimplicialComplex, Geometry, S<SimplicialComplex>>(m, "SimplicialComplex");


    auto class_VertexAttributes =
        py::class_<SimplicialComplex::VertexAttributes>(class_SimplicialComplex,
                                                        "VertexAttributes");

    auto class_EdgeAttributes =
        py::class_<SimplicialComplex::EdgeAttributes>(class_SimplicialComplex, "EdgeAttributes");

    auto class_TriangleAttributes =
        py::class_<SimplicialComplex::TriangleAttributes>(class_SimplicialComplex,
                                                          "TriangleAttributes");

    auto class_TetrahedronAttributes =
        py::class_<SimplicialComplex::TetrahedronAttributes>(class_SimplicialComplex,
                                                             "TetrahedronAttributes");

    // Method Def

    // NOTE: Don't allow python frontend to construct a SimplicialComplex directly
    // class_SimplicialComplex.def(py::init<>()); // removed

    class_SimplicialComplex.def(
        "transforms",
        [](SimplicialComplex& self) -> AttributeSlot<Matrix4x4>&
        { return self.transforms(); },
        py::return_value_policy::reference_internal);

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

    class_SimplicialComplex.def("copy",
                                [](SimplicialComplex& self) -> SimplicialComplex
                                { return self; });

    def_method(m, class_VertexAttributes);
    def_method(m, class_EdgeAttributes);
    def_method(m, class_TriangleAttributes);
    def_method(m, class_TetrahedronAttributes);
}
}  // namespace pyuipc::geometry
