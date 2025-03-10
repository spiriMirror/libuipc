#include <pyuipc/backend/module.h>
#include <pyuipc/backend/buffer_view.h>
#include <pyuipc/backend/scene_visitor.h>
#include <pyuipc/backend/world_visitor.h>
#include <pyuipc/backend/diff_sim_visitor.h>

namespace pyuipc::backend
{
PyModule::PyModule(py::module& m)
{
    PyDiffSimVisitor{m};
    PySceneVisitor{m};
    PyWorldVisitor{m};
}
}  // namespace pyuipc::backend
