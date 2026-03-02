#include <pyuipc/backend/animator_visitor.h>
#include <uipc/backend/visitors/animator_visitor.h>
#include <uipc/core/animator.h>

namespace pyuipc::backend
{
using namespace uipc::backend;
PyAnimatorVisitor::PyAnimatorVisitor(py::module_& m)
{
    py::class_<AnimatorVisitor>(m, "AnimatorVisitor", R"(AnimatorVisitor class for accessing animator data from backend.)")
        .def(py::init<uipc::core::Animator&>(), py::arg("animator"), R"(Create an AnimatorVisitor for an animator.)")
        .def("init", &AnimatorVisitor::init, R"(Initialize the animator visitor.)")
        .def("update", &AnimatorVisitor::update, R"(Update the animator visitor.)")
        .def("substep", &AnimatorVisitor::substep, R"(Get the current substep.)");
}
}  // namespace pyuipc::backend

