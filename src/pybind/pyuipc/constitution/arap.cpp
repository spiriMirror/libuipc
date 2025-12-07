#include <pyuipc/constitution/arap.h>
#include <uipc/constitution/arap.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyARAP::PyARAP(py::module& m)
{
    auto class_ARAP = py::class_<ARAP, FiniteElementConstitution>(m, "ARAP",
                                                                    R"(ARAP (As-Rigid-As-Possible) constitution for elastic deformation.)");

    class_ARAP.def(py::init<const Json&>(), py::arg("config") = ARAP::default_config(),
                  R"(Create an ARAP constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_ARAP.def_static("default_config", &ARAP::default_config,
                         R"(Get the default ARAP configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_ARAP.def("apply_to",
                   &ARAP::apply_to,
                   py::arg("sc"),
                   py::arg("kappa") = 1.0_MPa,
                   py::arg("mass_density") = 1.0e3,
                   R"(Apply ARAP constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    kappa: Stiffness parameter in MPa (default: 1.0 MPa).
    mass_density: Mass density (default: 1000.0).)");
}
}  // namespace pyuipc::constitution
