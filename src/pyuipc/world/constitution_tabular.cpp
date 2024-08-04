#include <pyuipc/world/constitution_tabular.h>
#include <uipc/world/constitution_tabular.h>
#include <uipc/constitutions/affine_body.h>
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

    class_ConstitutionTabular.def(
        "create",
        [](ConstitutionTabular& self, constitution::AffineBodyConstitution& affine_body) {
            return self.create<constitution::AffineBodyConstitution>(affine_body);
        });

    class_ConstitutionTabular.def("uids",
                                  [](ConstitutionTabular& self) {
                                      return as_numpy(self.uids(), py::cast(self));
                                  });

    class_ConstitutionTabular.def("types", &ConstitutionTabular::types);
}
}  // namespace pyuipc::world
