#include <pyuipc/constitution/empty.h>
#include <uipc/constitution/empty.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyEmpty::PyEmpty(py::module& m)
{
    auto class_Particle = py::class_<Empty, FiniteElementConstitution>(m, "Empty",
                                                                        R"(Empty constitution (no material properties, used for mass-only elements).)");

    class_Particle.def(py::init<const Json&>(), py::arg("config") = Empty::default_config(),
                       R"(Create an Empty constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_Particle.def_static("default_config", &Empty::default_config,
                             R"(Get the default Empty configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_Particle.def("apply_to",
                       &Empty::apply_to,
                       py::arg("sc"),
                       py::arg("mass_density") = 1000.0,
                       py::arg("thickness")    = 0.01_m,
                       R"(Apply Empty constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    mass_density: Mass density (default: 1000.0).
    thickness: Thickness in meters (default: 0.01 m).)");
}
}  // namespace pyuipc::constitution
