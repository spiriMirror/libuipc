#include <pyuipc/constitution/affine_body_shell.h>
#include <uipc/constitution/affine_body_shell.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyShell::PyAffineBodyShell(py::module& m)
{
    auto class_AffineBodyShell =
        py::class_<AffineBodyShell, AffineBodyConstitution>(
            m, "AffineBodyShell", R"(Codimensional 2D (shell) affine body constitution.

Convenience subclass of AffineBodyConstitution for open triangle meshes
treated as thin shells with a given thickness.)");

    class_AffineBodyShell.def(py::init<const Json&>(),
                              py::arg("config") =
                                  AffineBodyShell::default_config(),
                              R"(Create an AffineBodyShell.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyShell.def(
        "apply_to",
        &AffineBodyShell::apply_to,
        py::arg("sc"),
        py::arg("kappa"),
        py::arg("mass_density") = 1e3,
        py::arg("thickness")    = 0.01,
        R"(Apply the shell constitution to a 2D simplicial complex.
Args:
    sc: Triangle mesh (dim == 2).
    kappa: Stiffness parameter for shape energy.
    mass_density: Volume mass density in kg/m^3 (default: 1000.0).
    thickness: Shell thickness radius r; effective slab thickness = 2r (default: 0.01).)");
}
}  // namespace pyuipc::constitution


