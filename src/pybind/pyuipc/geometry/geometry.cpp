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

    class_Attribute.def(
        "find",
        [](Attributes& self, std::string_view name)
        { return Accessor::find(self, name); },
        py::arg("name"),
        R"(Find an instance attribute by name.
Args:
    name: Attribute name.
Returns:
    AttributeSlot or None: Attribute slot if found, None otherwise.)");

    class_Attribute.def(
        "resize",
        [](Attributes& self, size_t size) { std::move(self).resize(size); },
        py::arg("size"),
        R"(Resize the instance attributes to the specified size.
Args:
    size: New size for instance attributes.)");

    class_Attribute.def(
        "size",
        [](Attributes& self) { return std::move(self).size(); },
        R"(Get the number of instances.
Returns:
    int: Number of instances.)");

    class_Attribute.def(
        "reserve",
        [](Attributes& self, size_t size) { std::move(self).reserve(size); },
        py::arg("size"),
        R"(Reserve capacity for instance attributes.
Args:
    size: Capacity to reserve.)");

    class_Attribute.def(
        "clear", [](Attributes& self) { std::move(self).clear(); }, R"(Clear all instance attributes.)");

    class_Attribute.def(
        "destroy",
        [](Attributes& self, std::string_view name)
        { std::move(self).destroy(name); },
        py::arg("name"),
        R"(Destroy an instance attribute by name.
Args:
    name: Attribute name to destroy.)");


    class_Attribute.def(
        "share",
        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
        { Accessor::share(self, name, attribute); },
        py::arg("name"),
        py::arg("attribute"),
        R"(Share an existing attribute slot with a new name.
Args:
    name: New name for the shared attribute.
    attribute: Attribute slot to share.)");

    class_Attribute.def(
        "create",
        [](Attributes& self, std::string_view name, py::object object)
        { return Accessor::create(self, name, object); },
        py::arg("name"),
        py::arg("object"),
        R"(Create a new instance attribute from a Python object.
Args:
    name: Attribute name.
    object: Python object to create attribute from.
Returns:
    AttributeSlot: Created attribute slot.)");

    class_Attribute.def("to_json",
                        &Attributes::to_json,
                        R"(Convert instance attributes to JSON.
Returns:
    dict: JSON representation of instance attributes.)");
}

void def_method(py::module& m, py::class_<Geometry::MetaAttributes>& class_Attribute)
{
    using Attributes = Geometry::MetaAttributes;

    class_Attribute.def(
        "find",
        [](Attributes& self, std::string_view name)
        { return Accessor::find(self, name); },
        py::arg("name"),
        R"(Find a meta attribute by name.
Args:
    name: Attribute name.
Returns:
    AttributeSlot or None: Attribute slot if found, None otherwise.)");

    class_Attribute.def(
        "destroy",
        [](Attributes& self, std::string_view name)
        { std::move(self).destroy(name); },
        py::arg("name"),
        R"(Destroy a meta attribute by name.
Args:
    name: Attribute name to destroy.)");

    class_Attribute.def(
        "share",
        [](Attributes& self, std::string_view name, IAttributeSlot& attribute)
        { Accessor::share(self, name, attribute); },
        py::arg("name"),
        py::arg("attribute"),
        R"(Share an existing attribute slot with a new name.
Args:
    name: New name for the shared attribute.
    attribute: Attribute slot to share.)");

    class_Attribute.def(
        "create",
        [](Attributes& self, std::string_view name, py::object object)
        { return Accessor::create(self, name, object); },
        py::arg("name"),
        py::arg("object"),
        R"(Create a new meta attribute from a Python object.
Args:
    name: Attribute name.
    object: Python object to create attribute from.
Returns:
    AttributeSlot: Created attribute slot.)");

    class_Attribute.def("to_json",
                        &Attributes::to_json,
                        R"(Convert meta attributes to JSON.
Returns:
    dict: JSON representation of meta attributes.)");

    class_Attribute.def(
        "__repr__",
        [](const Attributes& self)
        { return fmt::format("{}", self.to_json().dump(4)); },
        R"(String representation of meta attributes.
Returns:
    str: Formatted JSON string.)");
}

}  // namespace pyuipc::geometry

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyGeometry::PyGeometry(py::module& m)
{
    // IGeometry:
    auto class_IGeometry =
        py::class_<IGeometry, S<IGeometry>>(m, "IGeometry", R"(Interface for geometry types.)");

    class_IGeometry.def("type",
                        &IGeometry::type,
                        R"(Get the geometry type name.
Returns:
    str: Geometry type name.)");

    class_IGeometry.def(
        "to_json",
        [](IGeometry& self) { return self.to_json(); },
        R"(Convert geometry to JSON representation.
Returns:
    dict: JSON representation of the geometry.)");

    class_IGeometry.def("clone",
                        &IGeometry::clone,
                        R"(Create a deep copy of the geometry.
Returns:
    IGeometry: Cloned geometry.)");


    // Geometry:
    auto class_Geometry = py::class_<Geometry, IGeometry, S<Geometry>>(
        m, "Geometry", R"(Geometry class representing geometric data with meta and instance attributes.)");

    class_Geometry.def(py::init<>(), R"(Create an empty geometry.)");

    auto class_MetaAttributes = py::class_<Geometry::MetaAttributes>(
        class_Geometry,
        "MetaAttributes",
        R"(Meta attributes associated with the geometry (shared across instances).)");

    auto class_InstanceAttributes = py::class_<Geometry::InstanceAttributes>(
        class_Geometry,
        "InstanceAttributes",
        R"(Instance attributes for geometry instances (per-instance data).)");


    class_Geometry.def(
        "meta",
        [](Geometry& self) { return self.meta(); },
        R"(Get the meta attributes.
Returns:
    MetaAttributes: Meta attributes of the geometry.)");

    class_Geometry.def(
        "instances",
        [](Geometry& self) { return self.instances(); },
        R"(Get the instance attributes.
Returns:
    InstanceAttributes: Instance attributes of the geometry.)");


    class_Geometry.def(
        "__repr__",
        [](Geometry& self) { return fmt::format("{}", self); },
        R"(String representation of the geometry.
Returns:
    str: String representation.)");


    class_Geometry.def(
        "__getitem__",
        [](Geometry& self, std::string_view name) { return self[name]; },
        py::arg("name"),
        R"(Get an attribute collection by name.
Returns:
    AttributeCollection: Attribute collection with the given name, if not found, create a new one.)");

    def_method(m, class_MetaAttributes);

    def_method(m, class_InstanceAttributes);
}
}  // namespace pyuipc::geometry
