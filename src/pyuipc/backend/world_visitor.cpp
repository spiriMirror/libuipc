#include <pyuipc/backend/world_visitor.h>
#include <uipc/backend/visitors/world_visitor.h>

namespace pyuipc::backend
{
using namespace uipc::backend;

PyWorldVisitor::PyWorldVisitor(py::module& m)
{
    auto class_WorldVisitor = py::class_<WorldVisitor>(m, "WorldVisitor");
    class_WorldVisitor.def("scene", &WorldVisitor::scene);
}
}  // namespace pyuipc::backend