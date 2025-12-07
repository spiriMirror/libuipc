#include <pyuipc/core/subscene_tabular.h>
#include <uipc/core/subscene_tabular.h>
#include <pyuipc/common/json.h>
#include <pyuipc/geometry/attribute_creator.h>

namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::core::PySubsceneTabular>
{
  public:
    static S<IAttributeSlot> create(core::SubsceneModelCollection& a,
                                    std::string_view               name,
                                    py::object                     object)
    {
        return pyuipc::geometry::AttributeCreator::create(a.m_attributes, name, object);
    }
};
}  // namespace uipc::geometry


namespace pyuipc::core
{
using namespace uipc::core;
PySubsceneTabular::PySubsceneTabular(py::module& m)
{
    {
        auto class_SubsceneElement = py::class_<SubsceneElement>(m, "SubsceneElement",
                                                                   R"(SubsceneElement class representing a subscene element (object or geometry).)");
        class_SubsceneElement.def(py::init<IndexT, std::string_view>(),
                                  py::arg("id"),
                                  py::arg("name"),
                                  R"(Create a subscene element.
Args:
    id: Element ID.
    name: Element name.)");
        class_SubsceneElement.def("id", &SubsceneElement::id,
                                  R"(Get the element ID.
Returns:
    int: Element ID.)");
        class_SubsceneElement.def("name", &SubsceneElement::name,
                                 R"(Get the element name.
Returns:
    str: Element name.)");
        class_SubsceneElement.def("apply_to", &SubsceneElement::apply_to,
                                 py::arg("object"),
                                 R"(Apply this subscene element to an object.
Args:
    object: Object to apply to.)");
    }

    {
        auto class_SubsceneModel = py::class_<SubsceneModel>(m, "SubsceneModel",
                                                              R"(SubsceneModel class representing subscene parameters between two elements.)");
        class_SubsceneModel.def("topo", &SubsceneModel::topo,
                               R"(Get the topology type.
Returns:
    str: Topology type string.)");
        class_SubsceneModel.def("is_enabled", &SubsceneModel::is_enabled,
                               R"(Check if the subscene model is enabled.
Returns:
    bool: True if enabled, False otherwise.)");
        class_SubsceneModel.def("config", &SubsceneModel::config,
                               R"(Get the configuration dictionary.
Returns:
    dict: Configuration dictionary.)");
    }


    {
        auto class_SubsceneModelCollection =
            py::class_<SubsceneModelCollection>(m, "SubsceneModelCollection",
                                                 R"(Collection of subscene models with attributes.)");

        using Accessor = uipc::geometry::AttributeFriend<PySubsceneTabular>;

        class_SubsceneModelCollection.def(
            "create",
            [](SubsceneModelCollection& self,
               std::string_view         name,
               py::object default_value) -> S<uipc::geometry::IAttributeSlot>
            { return Accessor::create(self, name, default_value); },
            py::arg("name"),
            py::arg("default_value"),
            R"(Create a subscene model attribute.
Args:
    name: Attribute name.
    default_value: Default value for the attribute.
Returns:
    AttributeSlot: Created attribute slot.)");

        class_SubsceneModelCollection.def("find",
                                          [](SubsceneModelCollection& self,
                                             std::string_view name) -> S<uipc::geometry::IAttributeSlot>
                                          { return self.find(name); },
                                          py::arg("name"),
                                          R"(Find a subscene model attribute by name.
Args:
    name: Attribute name.
Returns:
    AttributeSlot or None: Attribute slot if found, None otherwise.)");
        class_SubsceneModelCollection.def(
            "__repr__",
            [](const SubsceneModelCollection& self)
            { return fmt::format("{}", self.to_json().dump(4)); },
            R"(String representation of the collection.
Returns:
    str: Formatted JSON string.)");
    }

    {
        auto class_SubsceneTabular = py::class_<SubsceneTabular>(m, "SubsceneTabular",
                                                                  R"(SubsceneTabular class managing subscene elements and subscene models between them.)");

        // Elements:
        class_SubsceneTabular.def("create", &SubsceneTabular::create, py::arg("name") = "",
                                 R"(Create a new subscene element.
Args:
    name: Element name (optional).
Returns:
    SubsceneElement: Created subscene element.)");

        class_SubsceneTabular.def("default_element",
                                  &SubsceneTabular::default_element,
                                  py::return_value_policy::reference_internal,
                                  R"(Get the default subscene element.
Returns:
    SubsceneElement: Reference to default element.)");

        class_SubsceneTabular.def("element_count", &SubsceneTabular::element_count,
                                 R"(Get the number of subscene elements.
Returns:
    int: Number of elements.)");

        // Models:
        class_SubsceneTabular.def("insert",
                                  &SubsceneTabular::insert,
                                  py::arg("L"),
                                  py::arg("R"),
                                  py::arg("enable") = false,
                                  py::arg("config") = Json::object(),
                                  R"(Insert a subscene model between two elements.
Args:
    L: Left element ID.
    R: Right element ID.
    enable: Whether the subscene is enabled (default: False).
    config: Additional configuration dictionary (default: empty).)");

        class_SubsceneTabular.def(
            "at",
            [](SubsceneTabular& self, IndexT i, IndexT j) -> SubsceneModel
            { return self.at(i, j); },
            py::return_value_policy::move,
            py::arg("i"),
            py::arg("j"),
            R"(Get the subscene model between two elements.
Args:
    i: First element ID.
    j: Second element ID.
Returns:
    SubsceneModel: Subscene model between the two elements.)");

        class_SubsceneTabular.def(
            "subscene_models",
            [](SubsceneTabular& self) -> SubsceneModelCollection
            { return self.subscene_models(); },
            py::return_value_policy::move,
            R"(Get the subscene model collection.
Returns:
    SubsceneModelCollection: Collection of subscene models.)");
    }
}
}  // namespace pyuipc::core
