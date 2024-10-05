#include <pyuipc/core/module.h>
#include <pyuipc/core/engine.h>
#include <pyuipc/core/object.h>
#include <pyuipc/core/scene.h>
#include <pyuipc/core/world.h>
#include <pyuipc/core/contact_tabular.h>
#include <pyuipc/core/constitution_tabular.h>
#include <pyuipc/core/scene_io.h>
#include <pyuipc/core/animator.h>

namespace pyuipc::core
{
Module::Module(py::module& m)
{
    PyEngine{m};

    PyObject{m};

    PyContactTabular{m};
    PyConstitutionTabular{m};

    PyAnimator{m};
    PyScene{m};
    PyWorld{m};
    PySceneIO{m};
}
}  // namespace pyuipc::core
