#include <pyuipc/constitution/plastic_discrete_shell_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/plastic_discrete_shell_bending.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyPlasticDiscreteShellBending::PyPlasticDiscreteShellBending(py::module& m)
{
    auto class_PlasticDiscreteShellBending =
        py::class_<PlasticDiscreteShellBending, FiniteElementExtraConstitution>(
            m,
            "PlasticDiscreteShellBending",
            R"(PlasticDiscreteShellBending constitution for shell bending with plastic rest-angle evolution.)");

    class_PlasticDiscreteShellBending.def(
        py::init<const Json&>(),
        py::arg("config") = PlasticDiscreteShellBending::default_config(),
        R"(Create a PlasticDiscreteShellBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_PlasticDiscreteShellBending.def_static(
        "default_config",
        &PlasticDiscreteShellBending::default_config,
        R"(Get the default PlasticDiscreteShellBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_PlasticDiscreteShellBending.def(
        "apply_to",
        &PlasticDiscreteShellBending::apply_to,
        py::arg("sc"),
        py::arg("bending_stiffness"),
        py::arg("yield_threshold"),
        py::arg("hardening_modulus") = 0.0,
        R"(Apply PlasticDiscreteShellBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    bending_stiffness: Bending stiffness in kPa.
    yield_threshold: Yield threshold on dihedral angle deviation.
    hardening_modulus: Linear hardening modulus. Use 0.0 for perfect plasticity.)");
}
}  // namespace pyuipc::constitution
