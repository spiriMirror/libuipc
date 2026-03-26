#include <pyuipc/constitution/strain_plastic_discrete_shell_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/strain_plastic_discrete_shell_bending.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyStrainPlasticDiscreteShellBending::PyStrainPlasticDiscreteShellBending(py::module& m)
{
    auto class_StrainPlasticDiscreteShellBending =
        py::class_<StrainPlasticDiscreteShellBending, FiniteElementExtraConstitution>(
            m,
            "StrainPlasticDiscreteShellBending",
            R"(StrainPlasticDiscreteShellBending constitution for shell bending with strain-threshold plastic rest-angle evolution.)");

    class_StrainPlasticDiscreteShellBending.def(
        py::init<const Json&>(),
        py::arg("config") = StrainPlasticDiscreteShellBending::default_config(),
        R"(Create a StrainPlasticDiscreteShellBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_StrainPlasticDiscreteShellBending.def_static(
        "default_config",
        &StrainPlasticDiscreteShellBending::default_config,
        R"(Get the default StrainPlasticDiscreteShellBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_StrainPlasticDiscreteShellBending.def(
        "apply_to",
        &StrainPlasticDiscreteShellBending::apply_to,
        py::arg("sc"),
        py::arg("bending_stiffness"),
        py::arg("yield_threshold"),
        py::arg("hardening_modulus") = 0.0,
        R"(Apply StrainPlasticDiscreteShellBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    bending_stiffness: Bending stiffness in kPa.
    yield_threshold: Yield threshold on dihedral angle deviation.
    hardening_modulus: Linear hardening modulus. Use 0.0 for perfect plasticity.)");
}
}  // namespace pyuipc::constitution
