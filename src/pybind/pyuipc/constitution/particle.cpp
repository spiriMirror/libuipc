#include <pyuipc/constitution/particle.h>
#include <uipc/constitution/particle.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyParticle::PyParticle(py::module& m)
{
    auto class_Particle = py::class_<Particle, FiniteElementConstitution>(m, "Particle",
                                                                           R"(Particle constitution for point-mass simulation.)");

    class_Particle.def(py::init<const Json&>(),
                       py::arg("config") = Particle::default_config(),
                       R"(Create a Particle constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_Particle.def_static("default_config", &Particle::default_config,
                             R"(Get the default Particle configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_Particle.def("apply_to",
                       &Particle::apply_to,
                       py::arg("sc"),
                       py::arg("mass_density") = 1.0e3,
                       py::arg("thickness")    = 0.01_m,
                       R"(Apply Particle constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    mass_density: Mass density (default: 1000.0).
    thickness: Thickness in meters (default: 0.01 m).)");
}
}  // namespace pyuipc::constitution
