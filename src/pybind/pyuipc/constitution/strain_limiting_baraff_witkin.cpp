#include <pyuipc/constitution/strain_limiting_baraff_witkin.h>
#include <uipc/constitution/strain_limiting_baraff_witkin.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyStrainLimitingBaraffWitkinShell::PyStrainLimitingBaraffWitkinShell(py::module& m)
{
    auto class_StrainLimitingBaraffWitkinShell =
        py::class_<StrainLimitingBaraffWitkinShell, FiniteElementConstitution>(m, "StrainLimitingBaraffWitkinShell");

    class_StrainLimitingBaraffWitkinShell.def(py::init<const Json&>(),
                              py::arg("config") = StrainLimitingBaraffWitkinShell::default_config());

    class_StrainLimitingBaraffWitkinShell.def_static("default_config", &StrainLimitingBaraffWitkinShell::default_config);

    class_StrainLimitingBaraffWitkinShell.def("apply_to",
                              &StrainLimitingBaraffWitkinShell::apply_to,
                              py::arg("sc"),
                              py::arg("moduli") =
                                  ElasticModuli::youngs_poisson(1.0_MPa, 0.49),
                              py::arg("mass_density") = 2.0e2,
                              py::arg("thickness")    = 0.001_m);
}
}  // namespace pyuipc::constitution
