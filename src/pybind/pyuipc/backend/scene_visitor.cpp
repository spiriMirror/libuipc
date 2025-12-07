#include <pyuipc/backend/scene_visitor.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/core/scene.h>
#include <pyuipc/common/json.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/span.h>

namespace pyuipc::backend
{
using namespace uipc::backend;
using namespace uipc::core;
PySceneVisitor::PySceneVisitor(py::module& m)
{
    auto class_SceneVisitor = py::class_<SceneVisitor>(m, "SceneVisitor",
                                                        R"(SceneVisitor class for accessing scene data from backend.)");

    class_SceneVisitor.def(py::init<Scene&>(),
                          py::arg("scene"),
                          R"(Create a SceneVisitor for a scene.
Args:
    scene: Scene to visit.)");

    class_SceneVisitor.def("begin_pending", &SceneVisitor::begin_pending,
                          R"(Begin processing pending geometries.)");
    class_SceneVisitor.def("solve_pending", &SceneVisitor::solve_pending,
                          R"(Solve pending geometries.)");

    def_span<S<geometry::GeometrySlot>>(class_SceneVisitor, "GeometrySlotSpan");

    class_SceneVisitor.def("geometries",
                           [](SceneVisitor& self) { return self.geometries(); },
                           R"(Get all geometries.
Returns:
    GeometrySlotSpan: Span of geometry slots.)");
    class_SceneVisitor.def("pending_geometries",
                           [](SceneVisitor& self)
                           { return self.pending_geometries(); },
                           R"(Get pending geometries.
Returns:
    GeometrySlotSpan: Span of pending geometry slots.)");

    class_SceneVisitor.def("rest_geometries",
                           [](SceneVisitor& self)
                           { return self.rest_geometries(); },
                           R"(Get rest geometries.
Returns:
    GeometrySlotSpan: Span of rest geometry slots.)");

    class_SceneVisitor.def("pending_rest_geometries",
                           [](SceneVisitor& self)
                           { return self.pending_rest_geometries(); },
                           R"(Get pending rest geometries.
Returns:
    GeometrySlotSpan: Span of pending rest geometry slots.)");

    class_SceneVisitor.def("config", &SceneVisitor::config,
                          R"(Get the scene configuration.
Returns:
    dict: Configuration dictionary.)");

    class_SceneVisitor.def(
        "constitution_tabular",
        [](SceneVisitor& self) -> ConstitutionTabular&
        { return self.constitution_tabular(); },
        py::return_value_policy::reference_internal,
        R"(Get the constitution tabular.
Returns:
    ConstitutionTabular: Reference to constitution tabular.)");
    class_SceneVisitor.def(
        "contact_tabular",
        [](SceneVisitor& self) -> ContactTabular&
        { return self.contact_tabular(); },
        py::return_value_policy::reference_internal,
        R"(Get the contact tabular.
Returns:
    ContactTabular: Reference to contact tabular.)");

    class_SceneVisitor.def("diff_sim",
                           [](SceneVisitor& self) -> DiffSimVisitor&
                           { return self.diff_sim(); },
                           R"(Get the differential simulator visitor.
Returns:
    DiffSimVisitor: Reference to diff sim visitor.)");

    class_SceneVisitor.def("get", &SceneVisitor::get, py::return_value_policy::move,
                          R"(Get the scene.
Returns:
    Scene: Scene object.)");
}
}  // namespace pyuipc::backend