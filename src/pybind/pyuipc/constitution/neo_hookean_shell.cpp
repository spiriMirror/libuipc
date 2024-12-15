#include <pyuipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyNeoHookeanShell::PyNeoHookeanShell(py::module& m)
{
    auto class_NeoHookeanShell =
        py::class_<NeoHookeanShell, FiniteElementConstitution>(m, "NeoHookeanShell");

    class_NeoHookeanShell.def(py::init<const Json&>(),
                              py::arg("config") = NeoHookeanShell::default_config());

    class_NeoHookeanShell.def_static("default_config", &NeoHookeanShell::default_config);

    class_NeoHookeanShell.def("apply_to",
                              &NeoHookeanShell::apply_to,
                              py::arg("sc"),
                              py::arg("moduli") =
                                  ElasticModuli::youngs_poisson(10.0_MPa, 0.49),
                              py::arg("mass_density") = 1.0e3,
                              py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
