#include <pyuipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyConstitution::PyAffineBodyConstitution(py::module& m)
{
    auto class_AffineBodyConstitution = py::class_<AffineBodyConstitution, IConstitution>(
        m, "AffineBodyConstitution", R"(AffineBodyConstitution for rigid body simulation with affine transformations.)");

    class_AffineBodyConstitution.def(py::init<const Json&>(),
                                     py::arg("config") =
                                         AffineBodyConstitution::default_config(),
                                     R"(Create an AffineBodyConstitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyConstitution.def_static("default_config",
                                            &AffineBodyConstitution::default_config,
                                            R"(Get the default AffineBodyConstitution configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_AffineBodyConstitution.def(
        "apply_to",
        [](const AffineBodyConstitution& self, geometry::SimplicialComplex& sc, Float kappa, Float mass_density)
        { self.apply_to(sc, kappa, mass_density); },
        py::arg("sc"),
        py::arg("kappa"),
        py::arg("mass_density") = 1000.0,
        R"(Apply AffineBodyConstitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    kappa: Stiffness parameter.
    mass_density: Mass density (default: 1000.0).)");

    class_AffineBodyConstitution.def(
        "apply_to",
        [](const AffineBodyConstitution& self,
           geometry::SimplicialComplex&  sc,
           Float                         kappa,
           py::array_t<Float>            mass,
           Float                         volume)
        { self.apply_to(sc, kappa, to_matrix<Matrix12x12>(mass), volume); },
        py::arg("sc"),
        py::arg("kappa"),
        py::arg("mass"),
        py::arg("volume"),
        R"(Apply AffineBodyConstitution with explicit mass matrix and volume override.
Args:
    sc: SimplicialComplex to apply to.
    kappa: Stiffness parameter.
    mass: 12x12 ABD mass matrix.
    volume: Volume of the body.)");

    class_AffineBodyConstitution.def(
        "create_proxy",
        [](const AffineBodyConstitution& self,
           Float                         kappa,
           Float                         mass,
           py::array_t<Float>            mass_center,
           py::array_t<Float>            inertia,
           Float                         volume)
        {
            return self.create_proxy(kappa, mass, to_matrix<Vector3>(mass_center),
                                     to_matrix<Matrix3x3>(inertia), volume);
        },
        py::arg("kappa"),
        py::arg("mass"),
        py::arg("mass_center"),
        py::arg("inertia"),
        py::arg("volume"),
        R"(Create a 1-vertex proxy affine body from rigid body mass properties.
Args:
    kappa: Stiffness parameter.
    mass: Scalar mass.
    mass_center: (3,) center of mass.
    inertia: (3,3) inertia matrix.
    volume: Volume of the body.
Returns:
    SimplicialComplex: A single-vertex proxy geometry with ABD attributes.)");

    class_AffineBodyConstitution.def(
        "create_proxy",
        [](const AffineBodyConstitution& self,
           Float                         kappa,
           py::array_t<Float>            abd_mass,
           Float                         volume)
        {
            return self.create_proxy(kappa, to_matrix<Matrix12x12>(abd_mass), volume);
        },
        py::arg("kappa"),
        py::arg("abd_mass"),
        py::arg("volume"),
        R"(Create a 1-vertex proxy affine body from a precomputed 12x12 ABD mass matrix.
Args:
    kappa: Stiffness parameter.
    abd_mass: (12,12) ABD mass matrix.
    volume: Volume of the body.
Returns:
    SimplicialComplex: A single-vertex proxy geometry with ABD attributes.)");
}
}  // namespace pyuipc::constitution
