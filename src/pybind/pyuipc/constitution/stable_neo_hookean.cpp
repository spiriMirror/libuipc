#include <pyuipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyStableNeoHookean::PyStableNeoHookean(py::module& m)
{
    auto class_StableNeoHookean =
        py::class_<StableNeoHookean, FiniteElementConstitution>(m, "StableNeoHookean",
                                                                 R"(StableNeoHookean constitution for stable neo-Hookean hyperelastic material.)");

    class_StableNeoHookean.def(py::init<const Json&>(),
                               py::arg("config") = StableNeoHookean::default_config(),
                               R"(Create a StableNeoHookean constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_StableNeoHookean.def_static("default_config", &StableNeoHookean::default_config,
                                     R"(Get the default StableNeoHookean configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_StableNeoHookean.def("apply_to",
                               &StableNeoHookean::apply_to,
                               py::arg("sc"),
                               py::arg("moduli") =
                                   ElasticModuli::youngs_poisson(20.0_kPa, 0.49),
                               py::arg("mass_density") = 1.0e3,
                               R"(Apply StableNeoHookean constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: Elastic moduli (default: Young's modulus 20.0 kPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 1000.0).)");
}
}  // namespace pyuipc::constitution
