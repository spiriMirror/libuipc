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
        py::class_<DiscreteShellBending, FiniteElementExtraConstitution>(m, "DiscreteShellBending");

    class_DiscreteShellBending.def(py::init<const Json&>(),
                                   py::arg("config") =
                                       DiscreteShellBending::default_config());

    class_DiscreteShellBending.def_static("default_config",
                                          &DiscreteShellBending::default_config);

    class_DiscreteShellBending.def(
        "apply_to", &DiscreteShellBending::apply_to, py::arg("sc"), py::arg("E") = 100.0_kPa);
}
}  // namespace pyuipc::constitution
