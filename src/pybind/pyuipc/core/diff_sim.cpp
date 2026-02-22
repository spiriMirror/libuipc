#include <pyuipc/core/diff_sim.h>
#include <uipc/core/diff_sim.h>

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::diff_sim;
PyDiffSim::PyDiffSim(py::module_& m)
{
    auto class_DiffSim = py::class_<DiffSim>(
        m, "DiffSim", R"(DiffSim class for differential simulation parameters and state.)");

    class_DiffSim.def(
        "parameters",
        [](DiffSim& self) -> ParameterCollection&
        { return self.parameters(); },
        py::rv_policy::reference_internal,
        R"(Get the parameter collection.
Returns:
    Reference to the parameter collection.)");
}
}  // namespace pyuipc::core
