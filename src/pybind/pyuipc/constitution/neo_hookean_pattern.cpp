#include <pyuipc/constitution/neo_hookean_pattern.h>
#include <pybind11/stl.h>
#include <uipc/constitution/neo_hookean_pattern.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyNeoHookeanPattern::PyNeoHookeanPattern(py::module& m)
{
    auto class_NeoHookeanPattern =
        py::class_<NeoHookeanPattern, FiniteElementConstitution>(m, "NeoHookeanPattern");

    class_NeoHookeanPattern.def(py::init<const Json&>(),
                              py::arg("config") = NeoHookeanPattern::default_config());

    class_NeoHookeanPattern.def_static("default_config", &NeoHookeanPattern::default_config);

    class_NeoHookeanPattern.def("apply_to",
                              &NeoHookeanPattern::apply_to,
                              py::arg("sc"),
                              py::arg("rest_shape"),
                              py::arg("moduli") =
                                  ElasticModuli::youngs_poisson(10.0_MPa, 0.49),
                              py::arg("mass_density") = 1.0e3,
                              py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
