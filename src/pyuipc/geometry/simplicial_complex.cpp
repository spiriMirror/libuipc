#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/pyuipc.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/attribute_friend.h>
#include <pyuipc/as_numpy.h>

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
        auto pyobj = py::cast(a.m_attributes);
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
    class_Attribute.def("capacity", &Attributes::reserve);
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
}


PySimplicialComplex::PySimplicialComplex(py::module& m)
{
    // Class Def

    auto class_SimplicialComplex =
        py::class_<SimplicialComplex, Geometry>(m, "SimplicialComplex");


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
        //class_VertexAttributes.def(
        //    "topo",
        //    [](SimplicialComplex::VertexAttributes& self) { return self.topo(); },
        //    py::return_value_policy::move);


        //m.def("view",
        //      [](SimplicialComplexTopo<VertexSlot>& self)
        //      { return as_numpy(view(std::move(self))); });

        def_method(m, class_VertexAttributes);

        class_SimplicalComplexVertexTopo.def(
            "view",
            [](SimplicialComplexTopo<VertexSlot>& self)
            { return as_numpy(std::move(self).view(), py::cast(self)); });

        class_SimplicalComplexVertexTopo.def(
            "share",
            [](SimplicialComplexTopo<VertexSlot>& self, SimplicialComplexTopo<VertexSlot>& other)
            { return std::move(self).share(std::move(other)); });

        class_SimplicalComplexVertexTopo.def("is_shared",
                                             [](SimplicialComplexTopo<VertexSlot>& self) {
                                                 return std::move(self).is_shared();
                                             });
    }

    // Edges
    {
        class_EdgeAttributes.def("find",
                                 [](SimplicialComplex::EdgeAttributes& self, std::string_view name)
                                 { return Accessor::find(self, name); });

        class_EdgeAttributes.def(
            "topo",
            [](SimplicialComplex::EdgeAttributes& self) { return self.topo(); },
            py::return_value_policy::move);

        class_SimplicalComplexEdgeTopo.def(
            "view",
            [](SimplicialComplexTopo<EdgeSlot>& self)
            { return as_numpy(std::move(self).view(), py::cast(self)); });

        m.def("view",
              [](SimplicialComplexTopo<EdgeSlot>& self)
              { return as_numpy(view(std::move(self)), py::cast(self)); });

        class_SimplicalComplexEdgeTopo.def(
            "share",
            [](SimplicialComplexTopo<EdgeSlot>& self, SimplicialComplexTopo<EdgeSlot>& other)
            { return std::move(self).share(std::move(other)); });
    }


    {
        class_TriangleAttributes.def("find",
                                     [](SimplicialComplex::TriangleAttributes& self,
                                        std::string_view name)
                                     { return Accessor::find(self, name); });


        class_TriangleAttributes.def(
            "topo",
            [](SimplicialComplex::TriangleAttributes& self)
            { return self.topo(); },
            py::return_value_policy::move);

        class_SimplicalComplexTriangleTopo.def(
            "view",
            [](SimplicialComplexTopo<TriangleSlot>& self)
            { return as_numpy(std::move(self).view(), py::cast(self)); });

        m.def("view",
              [](SimplicialComplexTopo<TriangleSlot>& self)
              { return as_numpy(view(std::move(self)), py::cast(self)); });

        class_SimplicalComplexTriangleTopo.def(
            "share",
            [](SimplicialComplexTopo<TriangleSlot>& self,
               SimplicialComplexTopo<TriangleSlot>& other)
            { return std::move(self).share(std::move(other)); });
    }

    {
        class_TetrahedronAttributes.def("find",
                                        [](SimplicialComplex::TetrahedronAttributes& self,
                                           std::string_view name)
                                        { return Accessor::find(self, name); });

        class_TetrahedronAttributes.def(
            "topo",
            [](SimplicialComplex::TetrahedronAttributes& self)
            { return self.topo(); },
            py::return_value_policy::move);

        class_SimplicalComplexTetrahedronTopo.def(
            "view",
            [](SimplicialComplexTopo<TetrahedronSlot>& self)
            { return as_numpy(std::move(self).view(), py::cast(self)); });

        m.def("view",
              [](SimplicialComplexTopo<TetrahedronSlot>& self)
              { return as_numpy(view(std::move(self)), py::cast(self)); });

        class_SimplicalComplexTetrahedronTopo.def(
            "share",
            [](SimplicialComplexTopo<TetrahedronSlot>& self,
               SimplicialComplexTopo<TetrahedronSlot>& other)
            { return std::move(self).share(std::move(other)); });
    }
}
}  // namespace pyuipc::geometry