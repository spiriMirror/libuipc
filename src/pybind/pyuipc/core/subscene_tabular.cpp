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
        auto class_SubsceneElement = py::class_<SubsceneElement>(m, "SubsceneElement");
        class_SubsceneElement.def(py::init<IndexT, std::string_view>(),
                                  py::arg("id"),
                                  py::arg("name"));
        class_SubsceneElement.def("id", &SubsceneElement::id);
        class_SubsceneElement.def("name", &SubsceneElement::name);
        class_SubsceneElement.def("apply_to", &SubsceneElement::apply_to);
    }

    {
        auto class_SubsceneModel = py::class_<SubsceneModel>(m, "SubsceneModel");
        class_SubsceneModel.def("topo", &SubsceneModel::topo);
        class_SubsceneModel.def("is_enabled", &SubsceneModel::is_enabled);
        class_SubsceneModel.def("config", &SubsceneModel::config);
    }


    {
        auto class_SubsceneModelCollection =
            py::class_<SubsceneModelCollection>(m, "SubsceneModelCollection");

        using Accessor = uipc::geometry::AttributeFriend<PySubsceneTabular>;

        class_SubsceneModelCollection.def(
            "create",
            [](SubsceneModelCollection& self,
               std::string_view         name,
               py::object default_value) -> S<uipc::geometry::IAttributeSlot>
            { return Accessor::create(self, name, default_value); },
            py::arg("name"),
            py::arg("default_value"));

        class_SubsceneModelCollection.def("find",
                                          [](SubsceneModelCollection& self,
                                             std::string_view name) -> S<uipc::geometry::IAttributeSlot>
                                          { return self.find(name); });
    }

    {
        auto class_SubsceneTabular = py::class_<SubsceneTabular>(m, "SubsceneTabular");

        // Elements:
        class_SubsceneTabular.def("create", &SubsceneTabular::create, py::arg("name") = "");

        class_SubsceneTabular.def("default_element",
                                  &SubsceneTabular::default_element,
                                  py::return_value_policy::reference_internal);

        class_SubsceneTabular.def("element_count", &SubsceneTabular::element_count);

        // Models:
        class_SubsceneTabular.def("insert",
                                  &SubsceneTabular::insert,
                                  py::arg("L"),
                                  py::arg("R"),
                                  py::arg("enable") = true,
                                  py::arg("config") = Json::object());


        class_SubsceneTabular.def(
            "default_model",
            [](SubsceneTabular& self, bool enable, const Json& config)
            { self.default_model(enable, config); },
            py::arg("enable") = true,
            py::arg("config") = Json::object());

        class_SubsceneTabular.def(
            "default_model",
            [](SubsceneTabular& self) -> SubsceneModel
            { return self.default_model(); },
            py::return_value_policy::move);

        class_SubsceneTabular.def(
            "at",
            [](SubsceneTabular& self, IndexT i, IndexT j) -> SubsceneModel
            { return self.at(i, j); },
            py::return_value_policy::move);

        class_SubsceneTabular.def(
            "subscene_models",
            [](SubsceneTabular& self) -> SubsceneModelCollection
            { return self.subscene_models(); },
            py::return_value_policy::move);
    }
}
}  // namespace pyuipc::core
