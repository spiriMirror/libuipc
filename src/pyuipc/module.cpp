#include <pyuipc/pyuipc.h>
#include <pyuipc/geometry/module.h>
#include <pyuipc/world/module.h>
#include <pyuipc/engine/module.h>
#include <pyuipc/constitutions/module.h>
#include <uipc/common/uipc.h>
#include <pyuipc/common/json.h>
#include <pyuipc/world/constitution.h>
using namespace uipc;

PYBIND11_MODULE(pyuipc, m)
{
    m.doc() = "Libuipc Python Binding";

    m.def("init", &uipc::init);

    m.def("default_config", &uipc::default_config);
    m.def("config", &uipc::config);

    auto geometry = m.def_submodule("geometry");
    pyuipc::geometry::Module{geometry};

    auto engine = m.def_submodule("engine");
    pyuipc::engine::Module{engine};

    auto world = m.def_submodule("world");

    auto constitution = m.def_submodule("constitution");
    pyuipc::world::PyConstitution{world};  // must be before
    pyuipc::constitution::Module{constitution};


    pyuipc::world::Module{world};
}