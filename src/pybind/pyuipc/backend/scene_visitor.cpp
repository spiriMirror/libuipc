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
    auto class_SceneVisitor = py::class_<SceneVisitor>(m, "SceneVisitor");

    class_SceneVisitor.def(py::init<Scene&>());

    class_SceneVisitor.def("begin_pending", &SceneVisitor::begin_pending);
    class_SceneVisitor.def("solve_pending", &SceneVisitor::solve_pending);

    def_span<S<geometry::GeometrySlot>>(class_SceneVisitor, "GeometrySlotSpan");

    class_SceneVisitor.def("geometries",
                           [](SceneVisitor& self) { return self.geometries(); });
    class_SceneVisitor.def("pending_geometries",
                           [](SceneVisitor& self)
                           { return self.pending_geometries(); });

    class_SceneVisitor.def("rest_geometries",
                           [](SceneVisitor& self)
                           { return self.rest_geometries(); });

    class_SceneVisitor.def("pending_rest_geometries",
                           [](SceneVisitor& self)
                           { return self.pending_rest_geometries(); });

    class_SceneVisitor.def("config", &SceneVisitor::config);

    class_SceneVisitor.def(
        "constitution_tabular",
        [](SceneVisitor& self) -> ConstitutionTabular&
        { return self.constitution_tabular(); },
        py::return_value_policy::reference_internal);
    class_SceneVisitor.def(
        "contact_tabular",
        [](SceneVisitor& self) -> ContactTabular&
        { return self.contact_tabular(); },
        py::return_value_policy::reference_internal);

    class_SceneVisitor.def("diff_sim",
                           [](SceneVisitor& self) -> DiffSimVisitor&
                           { return self.diff_sim(); });

    class_SceneVisitor.def("get", &SceneVisitor::get, py::return_value_policy::move);
}
}  // namespace pyuipc::backend