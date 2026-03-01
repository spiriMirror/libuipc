#include <pyuipc/constitution/affine_body_rod.h>
#include <uipc/constitution/affine_body_rod.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyRod::PyAffineBodyRod(py::module& m)
{
    auto class_AffineBodyRod =
        py::class_<AffineBodyRod, AffineBodyConstitution>(
            m, "AffineBodyRod", R"(Codimensional 1D (rod) affine body constitution.

Convenience subclass of AffineBodyConstitution for edge meshes (polylines)
treated as thin rods with a circular cross-section of a given radius.)");

    class_AffineBodyRod.def(py::init<const Json&>(),
                            py::arg("config") =
                                AffineBodyRod::default_config(),
                            R"(Create an AffineBodyRod.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyRod.def(
        "apply_to",
        &AffineBodyRod::apply_to,
        py::arg("sc"),
        py::arg("kappa"),
        py::arg("mass_density") = 1e3,
        py::arg("thickness")    = 0.01,
        R"(Apply the rod constitution to a 1D simplicial complex.
Args:
    sc: Edge mesh (dim == 1).
    kappa: Stiffness parameter for shape energy.
    mass_density: Volume mass density in kg/m^3 (default: 1000.0).
    thickness: Rod cross-section radius r; cross-section area = pi * r^2 (default: 0.01).)");
}
}  // namespace pyuipc::constitution


