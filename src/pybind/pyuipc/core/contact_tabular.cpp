#include <pyuipc/core/contact_tabular.h>
#include <uipc/core/contact_tabular.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyContactTabular::PyContactTabular(py::module& m)
{
    auto class_ContactElement = py::class_<ContactElement>(m, "ContactElement");

    class_ContactElement.def(py::init<>());
    class_ContactElement.def(
        py::init<IndexT, std::string_view>(), py::arg("id"), py::arg("name"));
    class_ContactElement.def("id", &ContactElement::id);
    class_ContactElement.def("name", &ContactElement::name);
    class_ContactElement.def("apply_to", &ContactElement::apply_to);


    auto class_ContactTabular = py::class_<ContactTabular>(m, "ContactTabular");

    class_ContactTabular.def(py::init<>());
    class_ContactTabular.def("create", &ContactTabular::create, py::arg("name") = "");
    class_ContactTabular.def("insert",
                             &ContactTabular::insert,
                             py::arg("L"),
                             py::arg("R"),
                             py::arg("friction_rate"),
                             py::arg("resistance"),
                             py::arg("config") = Json::object());

    class_ContactTabular.def(
        "default_model",
        [](ContactTabular& self, Float friction_rate, Float resistance, const Json& config = Json{})
        { self.default_model(friction_rate, resistance, config); },
        py::arg("friction_rate"),
        py::arg("resistance"),
        py::arg("config") = Json::object());

    class_ContactTabular.def("default_element",
                             &ContactTabular::default_element,
                             py::return_value_policy::reference_internal);

    class_ContactTabular.def("element_count", &ContactTabular::element_count);
}
}  // namespace pyuipc::core
