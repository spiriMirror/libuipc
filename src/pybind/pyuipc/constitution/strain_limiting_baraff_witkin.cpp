#include <pyuipc/constitution/strain_limiting_baraff_witkin.h>
#include <uipc/constitution/strain_limiting_baraff_witkin.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
using namespace uipc::geometry;
PyStrainLimitingBaraffWitkinShell::PyStrainLimitingBaraffWitkinShell(py::module& m)
{
    auto class_StrainLimitingBaraffWitkinShell =
        py::class_<StrainLimitingBaraffWitkinShell, FiniteElementConstitution>(
            m,
            "StrainLimitingBaraffWitkinShell",
            R"(StrainLimitingBaraffWitkinShell constitution with strain limiting (Baraff-Witkin method).)");

    class_StrainLimitingBaraffWitkinShell.def(
        py::init<const Json&>(),
        py::arg("config") = StrainLimitingBaraffWitkinShell::default_config(),
        R"(Create a StrainLimitingBaraffWitkinShell constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_StrainLimitingBaraffWitkinShell.def_static("default_config",
                                                     &StrainLimitingBaraffWitkinShell::default_config,
                                                     R"(Get the default StrainLimitingBaraffWitkinShell configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_StrainLimitingBaraffWitkinShell.def(
        "apply_to",
        [](StrainLimitingBaraffWitkinShell& self,
           SimplicialComplex& sc,
           const ElasticModuli2D& moduli,
           Float mass_density,
           Float thickness)
        { self.apply_to(sc, moduli, mass_density, thickness); },
        py::arg("sc"),
        py::arg("moduli")       = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        py::arg("mass_density") = 2.0e2,
        py::arg("thickness")    = 0.001_m,
        R"(Apply StrainLimitingBaraffWitkinShell constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: ElasticModuli (default: Young's modulus 1.0 MPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 200.0).
    thickness: Shell thickness in meters (default: 0.001 m).)");
}
}  // namespace pyuipc::constitution
