#include <pyuipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyConstitution::PyAffineBodyConstitution(py::module& m)
{
    auto class_AffineBodyConstitution =
        py::class_<AffineBodyConstitution, IConstitution>(m, "AffineBodyConstitution",
                                                            R"(AffineBodyConstitution for rigid body simulation with affine transformations.)");

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

    class_AffineBodyConstitution.def("apply_to",
                                     &AffineBodyConstitution::apply_to,
                                     py::arg("sc"),
                                     py::arg("kappa"),
                                     py::arg("mass_density") = 1000.0,
                                     R"(Apply AffineBodyConstitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    kappa: Stiffness parameter.
    mass_density: Mass density (default: 1000.0).)");
}
}  // namespace pyuipc::constitution
