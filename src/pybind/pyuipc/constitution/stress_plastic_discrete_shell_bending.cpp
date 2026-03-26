#include <pyuipc/constitution/stress_plastic_discrete_shell_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/stress_plastic_discrete_shell_bending.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyStressPlasticDiscreteShellBending::PyStressPlasticDiscreteShellBending(py::module& m)
{
    auto class_StressPlasticDiscreteShellBending =
        py::class_<StressPlasticDiscreteShellBending, FiniteElementExtraConstitution>(
            m,
            "StressPlasticDiscreteShellBending",
            R"(StressPlasticDiscreteShellBending constitution for stress-based ECI shell bending plasticity.)");

    class_StressPlasticDiscreteShellBending.def(
        py::init<const Json&>(),
        py::arg("config") = StressPlasticDiscreteShellBending::default_config(),
        R"(Create a StressPlasticDiscreteShellBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_StressPlasticDiscreteShellBending.def_static(
        "default_config",
        &StressPlasticDiscreteShellBending::default_config,
        R"(Get the default StressPlasticDiscreteShellBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_StressPlasticDiscreteShellBending.def(
        "apply_to",
        &StressPlasticDiscreteShellBending::apply_to,
        py::arg("sc"),
        py::arg("bending_stiffness"),
        py::arg("yield_stress"),
        py::arg("hardening_modulus") = 0.0,
        R"(Apply StressPlasticDiscreteShellBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    bending_stiffness: Bending stiffness in kPa.
    yield_stress: Yield stress on the generalized bending moment.
    hardening_modulus: Isotropic hardening modulus in stress space. Use 0.0 for perfect plasticity.)");
}
}  // namespace pyuipc::constitution
