#include <pyuipc/world/module.h>
#include <pyuipc/world/object.h>
#include <pyuipc/world/scene.h>
#include <pyuipc/world/world.h>
#include <pyuipc/world/constitution.h>

namespace pyuipc::world
{
Module::Module(py::module& m)
{
    PyConstitution{m};
    PyObject{m};
    PyScene{m};
    PyWorld{m};
}
}  // namespace pyuipc::world
