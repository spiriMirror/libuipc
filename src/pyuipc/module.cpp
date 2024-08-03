#include <pyuipc/pyuipc.h>
#include <pyuipc/geometry/module.h>
#include <pyuipc/world/module.h>
#include <pyuipc/engine/module.h>
#include <pyuipc/constitutions/module.h>

PYBIND11_MODULE(pyuipc, m)
{
    m.doc() = "Libuipc Python Binding";

    auto geometry = m.def_submodule("geometry");
    pyuipc::geometry::Module{geometry};

    auto engine = m.def_submodule("engine");
    pyuipc::engine::Module{engine};

    auto constitution = m.def_submodule("constitution");
    pyuipc::constitution::Module{constitution};

    auto world = m.def_submodule("world");
    pyuipc::world::Module{world};
}