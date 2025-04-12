#include <pyuipc/backend/diff_sim_visitor.h>
#include <uipc/backend/visitors/diff_sim_visitor.h>
#include <uipc/core/diff_sim.h>

namespace pyuipc::backend
{
using namespace uipc::backend;
PyDiffSimVisitor::PyDiffSimVisitor(py::module& m)
{
    py::class_<DiffSimVisitor>(m, "DiffSimVisitor")
        .def(
            "parameters",
            [](DiffSimVisitor& self) -> diff_sim::ParameterCollection&
            { return self.parameters(); },
            py::return_value_policy::reference_internal)
        .def("ref", &DiffSimVisitor::ref, py::return_value_policy::reference_internal);
}
}  // namespace pyuipc::backend
