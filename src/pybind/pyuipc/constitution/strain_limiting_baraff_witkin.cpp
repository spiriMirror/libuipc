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
           SimplicialComplex&               sc,
           const ElasticModuli2D&           stretch_moduli,
           const ElasticModuli2D&           shear_moduli,
           Float                            mass_density,
           Float                            thickness,
           Float                            strain_rate)
        {
            self.apply_to(sc, stretch_moduli, shear_moduli, mass_density, thickness, strain_rate);
        },
        py::arg("sc"),
        py::arg("stretch_moduli"),
        py::arg("shear_moduli"),
        py::arg("mass_density") = 2.0e2,
        py::arg("thickness")    = 0.001_m,
        py::arg("strain_rate")  = 100.0,
        R"(Apply StrainLimitingBaraffWitkinShell constitution to a simplicial complex,
with independent stretch / shear material parameters.

Cloth stiffness convention: stretch = E_stretch*t/(1-nu_stretch^2),
shear = E_shear/(2*(1+nu_shear)) (thickness-independent); the backend
membrane measure is the triangle area, so the values are used literally.

Args:
    sc: SimplicialComplex to apply to.
    stretch_moduli: ElasticModuli2D for the stretch mode (e.g. ElasticModuli2D.youngs_poisson(1.0_MPa, 0.49)).
    shear_moduli: ElasticModuli2D for the shear mode (typically much softer, e.g. 1/100 of stretch).
    mass_density: Mass density (default: 200.0).
    thickness: Shell thickness in meters (default: 0.001 m).
    strain_rate: Baraff-Witkin over-stretch amplification rate (default: 100.0).)");

    class_StrainLimitingBaraffWitkinShell.def(
        "apply_to",
        [](StrainLimitingBaraffWitkinShell& self,
           SimplicialComplex&               sc,
           const ElasticModuli2D&           moduli,
           Float                            mass_density,
           Float                            thickness,
           Float                            strain_rate)
        { self.apply_to(sc, moduli, mass_density, thickness, strain_rate); },
        py::arg("sc"),
        py::arg("moduli") = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        py::arg("mass_density") = 2.0e2,
        py::arg("thickness")    = 0.001_m,
        py::arg("strain_rate")  = 100.0,
        R"(Apply StrainLimitingBaraffWitkinShell constitution to a simplicial complex,
sharing one moduli pair for stretch and shear.

Args:
    sc: SimplicialComplex to apply to.
    moduli: ElasticModuli (default: Young's modulus 1.0 MPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 200.0).
    thickness: Shell thickness in meters (default: 0.001 m).
    strain_rate: Baraff-Witkin over-stretch amplification rate (default: 100.0).)");
}
}  // namespace pyuipc::constitution
