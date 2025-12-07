#include <pyuipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyNeoHookeanShell::PyNeoHookeanShell(py::module& m)
{
    auto class_NeoHookeanShell =
        py::class_<NeoHookeanShell, FiniteElementConstitution>(m, "NeoHookeanShell",
                                                                R"(NeoHookeanShell constitution for neo-Hookean shell material.)");

    class_NeoHookeanShell.def(py::init<const Json&>(),
                              py::arg("config") = NeoHookeanShell::default_config(),
                              R"(Create a NeoHookeanShell constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_NeoHookeanShell.def_static("default_config", &NeoHookeanShell::default_config,
                                    R"(Get the default NeoHookeanShell configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_NeoHookeanShell.def("apply_to",
                              &NeoHookeanShell::apply_to,
                              py::arg("sc"),
                              py::arg("moduli") =
                                  ElasticModuli::youngs_poisson(10.0_MPa, 0.49),
                              py::arg("mass_density") = 1.0e3,
                              py::arg("thickness")    = 0.01_m,
                              R"(Apply NeoHookeanShell constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: Elastic moduli (default: Young's modulus 10.0 MPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 1000.0).
    thickness: Shell thickness in meters (default: 0.01 m).)");
}
}  // namespace pyuipc::constitution
