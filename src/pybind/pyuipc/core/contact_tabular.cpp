#include <pyuipc/core/contact_tabular.h>
#include <uipc/core/contact_tabular.h>
#include <pyuipc/common/json.h>
#include <pyuipc/geometry/attribute_creator.h>

namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::core::PyContactTabular>
{
  public:
    static S<IAttributeSlot> create(core::ContactModelCollection& a,
                                    std::string_view              name,
                                    py::object                    object)
    {
        return pyuipc::geometry::AttributeCreator::create(a.m_attributes, name, object);
    }
};
}  // namespace uipc::geometry


namespace pyuipc::core
{
using namespace uipc::core;
PyContactTabular::PyContactTabular(py::module& m)
{
    {
        auto class_ContactElement = py::class_<ContactElement>(m, "ContactElement",
                                                                 R"(ContactElement class representing a contact element (object or geometry).)");
        class_ContactElement.def(py::init<IndexT, std::string_view>(),
                                 py::arg("id"),
                                 py::arg("name"),
                                 R"(Create a contact element.
Args:
    id: Element ID.
    name: Element name.)");
        class_ContactElement.def("id", &ContactElement::id,
                                R"(Get the element ID.
Returns:
    int: Element ID.)");
        class_ContactElement.def("name", &ContactElement::name,
                                R"(Get the element name.
Returns:
    str: Element name.)");
        class_ContactElement.def("apply_to", &ContactElement::apply_to,
                                py::arg("object"),
                                R"(Apply this contact element to an object.
Args:
    object: Object to apply to.)");
    }

    {
        auto class_ContactModel = py::class_<ContactModel>(m, "ContactModel",
                                                            R"(ContactModel class representing contact parameters between two elements.)");
        class_ContactModel.def("topo", &ContactModel::topo,
                              R"(Get the topology type.
Returns:
    str: Topology type string.)");
        class_ContactModel.def("friction_rate", &ContactModel::friction_rate,
                              R"(Get the friction rate.
Returns:
    float: Friction rate value.)");
        class_ContactModel.def("resistance", &ContactModel::resistance,
                             R"(Get the resistance value.
Returns:
    float: Resistance value.)");
        class_ContactModel.def("is_enabled", &ContactModel::is_enabled,
                              R"(Check if the contact model is enabled.
Returns:
    bool: True if enabled, False otherwise.)");
        class_ContactModel.def("config", &ContactModel::config,
                             R"(Get the configuration dictionary.
Returns:
    dict: Configuration dictionary.)");
    }


    {
        auto class_ContactModelCollection =
            py::class_<ContactModelCollection>(m, "ContactModelCollection",
                                               R"(Collection of contact models with attributes.)");

        using Accessor = uipc::geometry::AttributeFriend<PyContactTabular>;

        class_ContactModelCollection.def(
            "create",
            [](ContactModelCollection& self,
               std::string_view        name,
               py::object default_value) -> S<uipc::geometry::IAttributeSlot>
            { return Accessor::create(self, name, default_value); },
            py::arg("name"),
            py::arg("default_value"),
            R"(Create a contact model attribute.
Args:
    name: Attribute name.
    default_value: Default value for the attribute.
Returns:
    AttributeSlot: Created attribute slot.)");

        class_ContactModelCollection.def("find",
                                         [](ContactModelCollection& self,
                                            std::string_view name) -> S<uipc::geometry::IAttributeSlot>
                                         { return self.find(name); },
                                         py::arg("name"),
                                         R"(Find a contact model attribute by name.
Args:
    name: Attribute name.
Returns:
    AttributeSlot or None: Attribute slot if found, None otherwise.)");

        class_ContactModelCollection.def(
            "__repr__",
            [](const ContactModelCollection& self) -> std::string
            { return fmt::format("{}", self.to_json().dump(4)); },
            R"(String representation of the collection.
Returns:
    str: Formatted JSON string.)");
    }

    {
        auto class_ContactTabular = py::class_<ContactTabular>(m, "ContactTabular",
                                                                 R"(ContactTabular class managing contact elements and contact models between them.)");

        // Elements:
        class_ContactTabular.def("create", &ContactTabular::create, py::arg("name") = "",
                                 R"(Create a new contact element.
Args:
    name: Element name (optional).
Returns:
    ContactElement: Created contact element.)");

        class_ContactTabular.def("default_element",
                                 &ContactTabular::default_element,
                                 py::return_value_policy::reference_internal,
                                 R"(Get the default contact element.
Returns:
    ContactElement: Reference to default element.)");

        class_ContactTabular.def("element_count", &ContactTabular::element_count,
                               R"(Get the number of contact elements.
Returns:
    int: Number of elements.)");

        // Models:
        class_ContactTabular.def("insert",
                                 &ContactTabular::insert,
                                 py::arg("L"),
                                 py::arg("R"),
                                 py::arg("friction_rate"),
                                 py::arg("resistance"),
                                 py::arg("enable") = true,
                                 py::arg("config") = Json::object(),
                                 R"(Insert a contact model between two elements.
Args:
    L: Left element ID.
    R: Right element ID.
    friction_rate: Friction rate value.
    resistance: Resistance value.
    enable: Whether the contact is enabled (default: True).
    config: Additional configuration dictionary (default: empty).)");

        class_ContactTabular.def(
            "default_model",
            [](ContactTabular& self, Float friction_rate, Float resistance, bool enable, const Json& config)
            { self.default_model(friction_rate, resistance, enable, config); },
            py::arg("friction_rate"),
            py::arg("resistance"),
            py::arg("enable") = true,
            py::arg("config") = Json::object(),
            R"(Set the default contact model parameters.
Args:
    friction_rate: Default friction rate.
    resistance: Default resistance value.
    enable: Whether contacts are enabled by default (default: True).
    config: Default configuration dictionary (default: empty).)");

        class_ContactTabular.def(
            "default_model",
            [](ContactTabular& self) -> ContactModel
            { return self.default_model(); },
            py::return_value_policy::move,
            R"(Get the default contact model.
Returns:
    ContactModel: Default contact model.)");

        class_ContactTabular.def(
            "at",
            [](ContactTabular& self, IndexT i, IndexT j) -> ContactModel
            { return self.at(i, j); },
            py::return_value_policy::move,
            py::arg("i"),
            py::arg("j"),
            R"(Get the contact model between two elements.
Args:
    i: First element ID.
    j: Second element ID.
Returns:
    ContactModel: Contact model between the two elements.)");

        class_ContactTabular.def(
            "contact_models",
            [](ContactTabular& self) -> ContactModelCollection
            { return self.contact_models(); },
            py::return_value_policy::move,
            R"(Get the contact model collection.
Returns:
    ContactModelCollection: Collection of contact models.)");
    }
}
}  // namespace pyuipc::core
