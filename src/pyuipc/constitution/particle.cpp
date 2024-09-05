#include <pyuipc/constitution/particle.h>
#include <uipc/constitution/particle.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyParticle::PyParticle(py::module& m)
{
    auto class_Particle = py::class_<Particle, FiniteElementConstitution>(m, "Particle");

    class_Particle.def(py::init<const Json&>(),
                       py::arg("config") = Particle::default_config());

    class_Particle.def_static("default_config", &Particle::default_config);

    class_Particle.def("apply_to",
                       &Particle::apply_to,
                       py::arg("sc"),
                       py::arg("mass_density") = 1.0e3,
                       py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
