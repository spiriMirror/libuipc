#include <pyuipc/geometry/geometry.h>
#include <uipc/geometry/geometry.h>
#include <pyuipc/common/json.h>
#include <uipc/geometry/attribute_friend.h>

namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::geometry::PyGeometry>
{
  public:
    static S<IAttributeSlot> find(Geometry::InstanceAttributes& a, std::string_view name)
    {
        return a.m_attributes.find(name);
    }

    static S<IAttributeSlot> find(Geometry::MetaAttributes& a, std::string_view name)
    {
        return a.m_attributes.find(name);
    }

    static void share(Geometry::InstanceAttributes& a, std::string_view name, IAttributeSlot& b)
    {
        a.m_attributes.share(name, b);
    }

    static void share(Geometry::MetaAttributes& a, std::string_view name, IAttributeSlot& b)
    {
        a.m_attributes.share(name, b);
    }

    static S<IAttributeSlot> create(Geometry::MetaAttributes& a,
                                    std::string_view          name,
                                    py::object                object)
    {
        auto pyobj = py::cast(a.m_attributes,
                              py::return_value_policy::reference_internal,  // member object is a reference in the parent object
                              py::cast(a)  // parent object
        );

        // call the create method of the member object
        return py::cast<S<IAttributeSlot>>(
            pyobj.attr("create").operator()(py::cast(name), object));
    }

    static S<IAttributeSlot> create(Geometry::InstanceAttributes& a,
                                    std::string_view              name,
                                    py::object                    object)
    {
        auto pyobj = py::cast(a.m_attributes,
                              py::return_value_policy::reference_internal,  // member object is a reference in the parent object
                              py::cast(a)  // parent object
        );

        // call the create method of the member object
        return py::cast<S<IAttributeSlot>>(
            pyobj.attr("create").operator()(py::cast(name), object));
    }
};
}  // namespace uipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

using Accessor = AttributeFriend<PyGeometry>;

void def_method(py::module& m, py::class_<Geometry::InstanceAttributes>& class_Attribute)
{
    using Attributes = Geometry::InstanceAttributes;

    class_Attribute.def("find",
                        [](Attributes& self, std::string_view name)
                        { return Accessor::find(self, name); });

    class_Attribute.def("resize",
                        [](Attributes& self, size_t size)
                        { std::move(self).resize(size); });

    class_Attribute.def("size",
                        [](Attributes& self) { return std::move(self).size(); });

    class_Attribute.def("reserve",
                        [](Attributes& self, size_t size)
                        { std::move(self).reserve(size); });

    class_Attribute.def("clear",
                        [](Attributes& self) { std::move(self).clear(); });

    class_Attribute.def("destroy",
                        [](Attributes& self, std::string_view name)
                        { std::move(self).destroy(name); });


    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
                        { Accessor::share(self, name, attribute); });

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object)
                        { return Accessor::create(self, name, object); });

    class_Attribute.def("to_json", &Attributes::to_json);
}

void def_method(py::module& m, py::class_<Geometry::MetaAttributes>& class_Attribute)
{
    using Attributes = Geometry::MetaAttributes;

    class_Attribute.def("find",
                        [](Attributes& self, std::string_view name)
                        { return Accessor::find(self, name); });

    class_Attribute.def("destroy",
                        [](Attributes& self, std::string_view name)
                        { std::move(self).destroy(name); });

    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
                        { Accessor::share(self, name, attribute); });

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object)
                        { return Accessor::create(self, name, object); });

    class_Attribute.def("to_json", &Attributes::to_json);
}

}  // namespace pyuipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyGeometry::PyGeometry(py::module& m)
{
    // IGeometry:
    auto class_IGeometry = py::class_<IGeometry, S<IGeometry>>(m, "IGeometry");

    class_IGeometry.def("type", &IGeometry::type);

    class_IGeometry.def("to_json", [](IGeometry& self) { return self.to_json(); });

    class_IGeometry.def("clone", &IGeometry::clone);


    // Geometry:
    auto class_Geometry = py::class_<Geometry, IGeometry, S<Geometry>>(m, "Geometry");

    class_Geometry.def(py::init<>());

    auto class_MetaAttributes =
        py::class_<Geometry::MetaAttributes>(class_Geometry, "MetaAttributes");

    auto class_InstanceAttributes =
        py::class_<Geometry::InstanceAttributes>(class_Geometry, "InstanceAttributes");


    class_Geometry.def("meta", [](Geometry& self) { return self.meta(); });

    class_Geometry.def("instances",
                       [](Geometry& self) { return self.instances(); });


    class_Geometry.def("__repr__",
                       [](Geometry& self) { return fmt::format("{}", self); });

    def_method(m, class_MetaAttributes);

    def_method(m, class_InstanceAttributes);
}
}  // namespace pyuipc::geometry
