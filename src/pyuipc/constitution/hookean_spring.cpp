#include <pyuipc/constitution/hookean_spring.h>
#include <uipc/constitution/hookean_spring.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyHookeanSpring::PyHookeanSpring(py::module& m)
{
    auto class_HookeanSpring =
        py::class_<HookeanSpring, FiniteElementConstitution>(m, "HookeanSpring");

    class_HookeanSpring.def(py::init<const Json&>(),
                            py::arg("config") = HookeanSpring::default_config());

    class_HookeanSpring.def_static("default_config", &HookeanSpring::default_config);

    class_HookeanSpring.def("apply_to",
                            &HookeanSpring::apply_to,
                            py::arg("sc"),
                            py::arg("moduli")       = 40.0_MPa,
                            py::arg("mass_density") = 1.0e3,
                            py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
