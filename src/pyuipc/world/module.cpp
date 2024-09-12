#include <pyuipc/world/module.h>
#include <pyuipc/world/object.h>
#include <pyuipc/world/scene.h>
#include <pyuipc/world/world.h>
#include <pyuipc/world/contact_tabular.h>
#include <pyuipc/world/constitution_tabular.h>
#include <pyuipc/world/scene_io.h>
#include <pyuipc/world/animator.h>

namespace pyuipc::world
{
Module::Module(py::module& m)
{
    PyObject{m};

    PyContactTabular{m};
    PyConstitutionTabular{m};

    PyAnimator{m};
    PyScene{m};
    PyWorld{m};
    PySceneIO{m};
}
}  // namespace pyuipc::world
