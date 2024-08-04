#include <pyuipc/world/module.h>
#include <pyuipc/world/object.h>
#include <pyuipc/world/scene.h>
#include <pyuipc/world/world.h>
#include <pyuipc/world/constitution.h>
#include <pyuipc/world/contact_tabular.h>
#include <pyuipc/world/constitution_tabular.h>
#include <pyuipc/world/scene_io.h>

namespace pyuipc::world
{
Module::Module(py::module& m)
{
    PyObject{m};

    PyContactTabular{m};
    PyConstitutionTabular{m};

    PyScene{m};
    PyWorld{m};
    PySceneIO{m};
}
}  // namespace pyuipc::world
