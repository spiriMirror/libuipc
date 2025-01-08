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
        auto class_ContactElement = py::class_<ContactElement>(m, "ContactElement");
        class_ContactElement.def(py::init<IndexT, std::string_view>(),
                                 py::arg("id"),
                                 py::arg("name"));
        class_ContactElement.def("id", &ContactElement::id);
        class_ContactElement.def("name", &ContactElement::name);
        class_ContactElement.def("apply_to", &ContactElement::apply_to);
    }

    {
        auto class_ContactModel = py::class_<ContactModel>(m, "ContactModel");
        class_ContactModel.def("topo", &ContactModel::topo);
        class_ContactModel.def("friction_rate", &ContactModel::friction_rate);
        class_ContactModel.def("resistance", &ContactModel::resistance);
        class_ContactModel.def("is_enabled", &ContactModel::is_enabled);
        class_ContactModel.def("config", &ContactModel::config);
    }


    {
        auto class_ContactModelCollection =
            py::class_<ContactModelCollection>(m, "ContactModelCollection");

        using Accessor = uipc::geometry::AttributeFriend<PyContactTabular>;

        class_ContactModelCollection.def(
            "create",
            [](ContactModelCollection& self,
               std::string_view        name,
               py::object default_value) -> S<uipc::geometry::IAttributeSlot>
            { return Accessor::create(self, name, default_value); },
            py::arg("name"),
            py::arg("default_value"));

        class_ContactModelCollection.def("find",
                                         [](ContactModelCollection& self,
                                            std::string_view name) -> S<uipc::geometry::IAttributeSlot>
                                         { return self.find(name); });
    }

    {
        auto class_ContactTabular = py::class_<ContactTabular>(m, "ContactTabular");

        // Elements:
        class_ContactTabular.def("create", &ContactTabular::create, py::arg("name") = "");

        class_ContactTabular.def("default_element",
                                 &ContactTabular::default_element,
                                 py::return_value_policy::reference_internal);

        class_ContactTabular.def("element_count", &ContactTabular::element_count);

        // Models:
        class_ContactTabular.def("insert",
                                 &ContactTabular::insert,
                                 py::arg("L"),
                                 py::arg("R"),
                                 py::arg("friction_rate"),
                                 py::arg("resistance"),
                                 py::arg("enable") = true,
                                 py::arg("config") = Json::object());


        class_ContactTabular.def(
            "default_model",
            [](ContactTabular& self, Float friction_rate, Float resistance, bool enable, const Json& config)
            { self.default_model(friction_rate, resistance, enable, config); },
            py::arg("friction_rate"),
            py::arg("resistance"),
            py::arg("enable") = true,
            py::arg("config") = Json::object());

        class_ContactTabular.def("default_model",
                                 [](ContactTabular& self) -> ContactModel
                                 { return self.default_model(); });

        class_ContactTabular.def("at",
                                 [](ContactTabular& self, IndexT i, IndexT j) -> ContactModel
                                 { return self.at(i, j); });

        class_ContactTabular.def("contact_models",
                                 [](ContactTabular& self) -> ContactModelCollection
                                 { return self.contact_models(); });
    }
}
}  // namespace pyuipc::core
