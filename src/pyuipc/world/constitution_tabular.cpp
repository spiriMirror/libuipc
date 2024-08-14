#include <pyuipc/world/constitution_tabular.h>
#include <uipc/world/constitution_tabular.h>
#include <uipc/constitution/affine_body.h>
#include <pybind11/stl.h>
#include <pyuipc/as_numpy.h>
namespace pyuipc::world
{
using namespace uipc::world;
PyConstitutionTabular::PyConstitutionTabular(py::module& m)
{
    auto class_ConstitutionTabular =
        py::class_<ConstitutionTabular>(m, "ConstitutionTabular");

    class_ConstitutionTabular.def(py::init<>());

    class_ConstitutionTabular.def("insert", &ConstitutionTabular::insert);

    class_ConstitutionTabular.def("uids",
                                  [](ConstitutionTabular& self) {
                                      return as_numpy(self.uids(), py::cast(self));
                                  });

    class_ConstitutionTabular.def("types", &ConstitutionTabular::types);
}
}  // namespace pyuipc::world
