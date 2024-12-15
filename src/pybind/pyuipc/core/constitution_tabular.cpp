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
        py::class_<ConstitutionTabular>(m, "ConstitutionTabular");

    class_ConstitutionTabular.def(py::init<>());

    class_ConstitutionTabular.def("insert",
                                  [](ConstitutionTabular& self, constitution::IConstitution& c)
                                  { self.insert(c); });

    class_ConstitutionTabular.def("uids",
                                  [](ConstitutionTabular& self) {
                                      return as_numpy(self.uids(), py::cast(self));
                                  });

    class_ConstitutionTabular.def("types", &ConstitutionTabular::types);
}
}  // namespace pyuipc::core
