#include <pyuipc/constitution/empty.h>
#include <uipc/constitution/empty.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyEmpty::PyEmpty(py::module& m)
{
    auto class_Particle = py::class_<Empty, FiniteElementConstitution>(m, "Empty");

    class_Particle.def(py::init<const Json&>(), py::arg("config") = Empty::default_config());

    class_Particle.def_static("default_config", &Empty::default_config);

    class_Particle.def("apply_to",
                       &Empty::apply_to,
                       py::arg("sc"),
                       py::arg("mass_density") = 1000.0,
                       py::arg("thickness")    = 0.01_m);
}
}  // namespace pyuipc::constitution
