#include <pyuipc/core/diff_sim.h>
#include <uipc/core/diff_sim.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyDiffSim::PyDiffSim(py::module& m)
{
    auto class_DiffSim = py::class_<DiffSim>(m, "DiffSim",
                                             R"(DiffSim class for differential simulation parameters and state.)");

    class_DiffSim.def(
        "parameters",
        [](DiffSim& self) -> uipc::diff_sim::ParameterCollection&
        { return self.parameters(); },
        py::return_value_policy::reference_internal,
        R"(Get the parameter collection.
Returns:
    ParameterCollection: Reference to the parameter collection.)");
}
}  // namespace pyuipc::core
