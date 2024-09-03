#include <pyuipc/constitution/shell_neo_hookean.h>
#include <uipc/constitution/shell_neo_hookean.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyShellNeoHookean::PyShellNeoHookean(py::module& m)
{
    auto class_ShellNeoHookean =
        py::class_<ShellNeoHookean, FiniteElementConstitution>(m, "ShellNeoHookean");

    class_ShellNeoHookean.def(py::init<const Json&>(),
                              py::arg("config") = ShellNeoHookean::default_config());

    class_ShellNeoHookean.def_static("default_config", &ShellNeoHookean::default_config);

    class_ShellNeoHookean.def("apply_to",
                              &ShellNeoHookean::apply_to,
                              py::arg("sc"),
                              py::arg("moduli") =
                                  ElasticModuli::youngs_poisson(10.0_MPa, 0.49),
                              py::arg("mass_density") = 1.0e3,
                              py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
