#include <pyuipc/backend/world_visitor.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>
namespace pyuipc::backend
{
using namespace uipc::backend;

PyWorldVisitor::PyWorldVisitor(py::module& m)
{
    auto class_WorldVisitor = py::class_<WorldVisitor>(m, "WorldVisitor");
    class_WorldVisitor.def(py::init<uipc::core::World&>());
    class_WorldVisitor.def("scene", &WorldVisitor::scene);
    class_WorldVisitor.def("animator", &WorldVisitor::animator);
    class_WorldVisitor.def("ref", &WorldVisitor::ref, py::return_value_policy::reference_internal);
}
}  // namespace pyuipc::backend