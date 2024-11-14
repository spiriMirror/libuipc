#include <pyuipc/core/diff_sim.h>
#include <uipc/core/diff_sim.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyDiffSim::PyDiffSim(py::module& m)
{
    auto class_DiffSim = py::class_<DiffSim>(m, "DiffSim");

    class_DiffSim.def(
        "parameters",
        [](DiffSim& self) -> uipc::diff_sim::ParameterCollection&
        { return self.parameters(); },
        py::return_value_policy::reference_internal);
    class_DiffSim.def("H", [](DiffSim& self) { return self.H(); });
    class_DiffSim.def("pGpP", [](DiffSim& self) { return self.pGpP(); });
    class_DiffSim.def("clear", [](DiffSim& self) { return self.clear(); });
}
}  // namespace pyuipc::core
