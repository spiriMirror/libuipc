#include <pyuipc/core/constitution_tabular.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <pybind11/stl.h>
#include <pyuipc/as_numpy.h>
namespace pyuipc::core
{
using namespace uipc::core;
PyConstitutionTabular::PyConstitutionTabular(py::module& m)
{
    auto class_ConstitutionTabular =
        py::class_<ConstitutionTabular>(m, "ConstitutionTabular",
                                         R"(ConstitutionTabular class managing constitutions (material models) in a scene.)");

    class_ConstitutionTabular.def(py::init<>(),
                                  R"(Create an empty constitution tabular.)");

    class_ConstitutionTabular.def("insert",
                                  [](ConstitutionTabular& self, constitution::IConstitution& c)
                                  { self.insert(c); },
                                  py::arg("constitution"),
                                  R"(Insert a constitution into the tabular.
Args:
    constitution: Constitution to insert.)");

    class_ConstitutionTabular.def("uids",
                                  [](ConstitutionTabular& self) {
                                      return as_numpy(self.uids(), py::cast(self));
                                  },
                                  R"(Get the UIDs of all constitutions.
Returns:
    numpy.ndarray: Array of constitution UIDs.)");

    class_ConstitutionTabular.def("types", &ConstitutionTabular::types,
                                  R"(Get the types of all constitutions.
Returns:
    list: List of constitution type names.)");
}
}  // namespace pyuipc::core
