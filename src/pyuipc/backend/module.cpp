#include <pyuipc/backend/module.h>
#include <pyuipc/backend/scene_visitor.h>
#include <pyuipc/backend/world_visitor.h>

namespace pyuipc::backend
{
Module::Module(py::module& m)
{
    PySceneVisitor{m};
    PyWorldVisitor{m};
}
}  // namespace pyuipc::backend