#include <pyuipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyDiscreteShellBending::PyDiscreteShellBending(py::module& m)
{
    auto class_DiscreteShellBending =
        py::class_<DiscreteShellBending, FiniteElementExtraConstitution>(m, "DiscreteShellBending",
                                                                           R"(DiscreteShellBending constitution for shell bending energy.)");

    class_DiscreteShellBending.def(py::init<const Json&>(),
                                   py::arg("config") =
                                       DiscreteShellBending::default_config(),
                                   R"(Create a DiscreteShellBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_DiscreteShellBending.def_static("default_config",
                                          &DiscreteShellBending::default_config,
                                          R"(Get the default DiscreteShellBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_DiscreteShellBending.def("apply_to",
                                   &DiscreteShellBending::apply_to,
                                   py::arg("sc"),
                                   py::arg("bending_stiffness") = 100.0_kPa,
                                   R"(Apply DiscreteShellBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    bending_stiffness: Bending stiffness in kPa (default: 100.0 kPa).)");
}
}  // namespace pyuipc::constitution
