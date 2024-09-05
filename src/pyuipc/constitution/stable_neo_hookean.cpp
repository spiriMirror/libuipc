#include <pyuipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyStableNeoHookean::PyStableNeoHookean(py::module& m)
{
    auto class_StableNeoHookean =
        py::class_<StableNeoHookean, FiniteElementConstitution>(m, "StableNeoHookean");

    class_StableNeoHookean.def(py::init<const Json&>(),
                               py::arg("config") = StableNeoHookean::default_config());

    class_StableNeoHookean.def_static("default_config", &StableNeoHookean::default_config);

    class_StableNeoHookean.def("apply_to",
                               &StableNeoHookean::apply_to,
                               py::arg("sc"),
                               py::arg("moduli") =
                                   ElasticModuli::youngs_poisson(20.0_kPa, 0.49),
                               py::arg("mass_density") = 1.0e3);
}
}  // namespace pyuipc::constitution
