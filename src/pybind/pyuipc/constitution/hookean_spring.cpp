#include <pyuipc/constitution/hookean_spring.h>
#include <uipc/constitution/hookean_spring.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyHookeanSpring::PyHookeanSpring(py::module& m)
{
    auto class_HookeanSpring =
        py::class_<HookeanSpring, FiniteElementConstitution>(m, "HookeanSpring",
                                                              R"(HookeanSpring constitution for linear elastic spring behavior.)");

    class_HookeanSpring.def(py::init<const Json&>(),
                            py::arg("config") = HookeanSpring::default_config(),
                            R"(Create a HookeanSpring constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_HookeanSpring.def_static("default_config", &HookeanSpring::default_config,
                                  R"(Get the default HookeanSpring configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_HookeanSpring.def("apply_to",
                            &HookeanSpring::apply_to,
                            py::arg("sc"),
                            py::arg("moduli")       = 40.0_MPa,
                            py::arg("mass_density") = 1.0e3,
                            py::arg("thickness")    = 0.01_m,
                            R"(Apply HookeanSpring constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: Elastic moduli in MPa (default: 40.0 MPa).
    mass_density: Mass density (default: 1000.0).
    thickness: Thickness in meters (default: 0.01 m).)");
}
}  // namespace pyuipc::constitution
