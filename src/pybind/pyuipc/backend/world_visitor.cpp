#include <pyuipc/backend/world_visitor.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>
namespace pyuipc::backend
{
using namespace uipc::backend;

PyWorldVisitor::PyWorldVisitor(py::module& m)
{
    auto class_WorldVisitor = py::class_<WorldVisitor>(m, "WorldVisitor",
                                                        R"(WorldVisitor class for accessing world data from backend.)");
    class_WorldVisitor.def(py::init<uipc::core::World&>(),
                          py::arg("world"),
                          R"(Create a WorldVisitor for a world.
Args:
    world: World to visit.)");
    class_WorldVisitor.def("scene", &WorldVisitor::scene,
                          R"(Get the scene visitor.
Returns:
    SceneVisitor: Scene visitor.)");
    class_WorldVisitor.def("animator", &WorldVisitor::animator,
                          R"(Get the animator.
Returns:
    Animator: Reference to animator.)");
    class_WorldVisitor.def("get", &WorldVisitor::get, py::return_value_policy::move,
                          R"(Get the world.
Returns:
    World: World object.)");
}
}  // namespace pyuipc::backend