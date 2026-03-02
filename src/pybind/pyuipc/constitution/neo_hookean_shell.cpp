#include <pyuipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/neo_hookean_shell.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
using namespace uipc::geometry;
PyNeoHookeanShell::PyNeoHookeanShell(py::module_& m)
{
    auto class_NeoHookeanShell = py::class_<NeoHookeanShell, FiniteElementConstitution>(
        m, "NeoHookeanShell", R"(NeoHookeanShell constitution for neo-Hookean shell material.)");

    class_NeoHookeanShell.def(py::init<const Json&>(),
                              py::arg("config") = NeoHookeanShell::default_config(),
                              R"(Create a NeoHookeanShell constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_NeoHookeanShell.def_static("default_config",
                                     &NeoHookeanShell::default_config,
                                     R"(Get the default NeoHookeanShell configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_NeoHookeanShell.def("apply_to",
                              [](NeoHookeanShell& self,
                                 SimplicialComplex& sc,
                                 const ElasticModuli2D& moduli,
                                 Float mass_density,
                                 Float thickness)
                              { self.apply_to(sc, moduli, mass_density, thickness); },
                              py::arg("sc"),
                              py::arg("moduli") =
                                  ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
                              py::arg("mass_density") = 2.0e2,
                              py::arg("thickness")    = 0.001_m,
                              R"(Apply NeoHookeanShell constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: ElasticModuli2D (default: Young's modulus 1.0 MPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 200.0).
    thickness: Shell thickness in meters (default: 0.001 m).)");
}
}  // namespace pyuipc::constitution