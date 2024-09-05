#include <pyuipc/pyuipc.h>
#include <uipc/common/uipc.h>
#include <uipc/common/log.h>
#include <pyuipc/common/json.h>
#include <pyuipc/common/uipc_type.h>
#include <pyuipc/common/transform.h>
#include <pyuipc/common/logger.h>
#include <pyuipc/geometry/module.h>
#include <pyuipc/world/module.h>
#include <pyuipc/engine/module.h>
#include <pyuipc/constitution/module.h>
#include <pyuipc/backend/module.h>
#include <pyuipc/builtin/module.h>


using namespace uipc;

PYBIND11_MODULE(pyuipc, m)
{
    m.doc() = "Libuipc Python Binding";

    m.def("init", &uipc::init);
    m.def("default_config", &uipc::default_config);
    m.def("config", &uipc::config);

    pyuipc::PyUIPCType{m};
    pyuipc::PyLogger{m};
    pyuipc::PyTransform{m};

    // pyuipc.geometry
    auto geometry = m.def_submodule("geometry");
    pyuipc::geometry::Module{geometry};

    // pyuipc.engine
    auto engine = m.def_submodule("engine");
    pyuipc::engine::Module{engine};
    m.attr("Engine") = engine.attr("Engine");

    // pyuipc.constitution
    auto constitution = m.def_submodule("constitution");
    pyuipc::constitution::Module{constitution};

    // pyuipc.world
    auto world = m.def_submodule("world");
    pyuipc::world::Module{world};
    m.attr("World")   = world.attr("World");
    m.attr("Scene")   = world.attr("Scene");
    m.attr("SceneIO") = world.attr("SceneIO");

    // pyuipc.backend
    auto backend = m.def_submodule("backend");
    pyuipc::backend::Module{backend};

    // pyuipc.builtin
    auto builtin = m.def_submodule("builtin");
    pyuipc::builtin::Module{builtin};
}