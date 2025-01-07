#include <pyuipc/constitution/arap.h>
#include <uipc/constitution/arap.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyARAP::PyARAP(py::module& m)
{
    auto class_ARAP = py::class_<ARAP, FiniteElementConstitution>(m, "ARAP");

    class_ARAP.def(py::init<const Json&>(), py::arg("config") = ARAP::default_config());

    class_ARAP.def_static("default_config", &ARAP::default_config);

    class_ARAP.def("apply_to",
                   &ARAP::apply_to,
                   py::arg("sc"),
                   py::arg("kappa") = 1.0_MPa,
                   py::arg("mass_density") = 1.0e3);
}
}  // namespace pyuipc::constitution
