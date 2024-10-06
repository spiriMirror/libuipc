#include <pyuipc/pyuipc.h>
#include <uipc/common/uipc.h>
#include <uipc/common/log.h>
#include <pyuipc/common/json.h>
#include <pyuipc/common/uipc_type.h>
#include <pyuipc/common/timer.h>
#include <pyuipc/common/transform.h>
#include <pyuipc/common/logger.h>
#include <pyuipc/geometry/module.h>
#include <pyuipc/core/module.h>
#include <pyuipc/constitution/module.h>
#include <pyuipc/backend/module.h>
#include <pyuipc/builtin/module.h>
#include <pyuipc/common/unit.h>

using namespace uipc;

PYBIND11_MODULE(pyuipc, m)
{
    // pyuipc
    m.doc() = "Libuipc Python Binding";

    m.def("init", &uipc::init);
    m.def("default_config", &uipc::default_config);
    m.def("config", &uipc::config);

    pyuipc::PyUIPCType{m};
    pyuipc::PyLogger{m};
    pyuipc::PyTransform{m};
    pyuipc::PyTimer{m};

    // pyuipc.unit
    auto unit = m.def_submodule("unit");
    pyuipc::PyUnit{unit};


    // pyuipc.geometry
    auto geometry = m.def_submodule("geometry");
    pyuipc::geometry::Module{geometry};

    // pyuipc.constitution
    auto constitution = m.def_submodule("constitution");
    pyuipc::constitution::Module{constitution};

    // pyuipc.core
    auto core = m.def_submodule("core");
    pyuipc::core::Module{core};
    // expose core classes to top level
    m.attr("Engine")  = core.attr("Engine");
    m.attr("World")   = core.attr("World");
    m.attr("Scene")   = core.attr("Scene");
    m.attr("SceneIO") = core.attr("SceneIO");

    // pyuipc.backend
    auto backend = m.def_submodule("backend");
    pyuipc::backend::Module{backend};

    // pyuipc.builtin
    auto builtin = m.def_submodule("builtin");
    pyuipc::builtin::Module{builtin};
}