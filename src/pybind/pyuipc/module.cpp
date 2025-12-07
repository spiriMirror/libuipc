#include <pyuipc/common/json.h>
#include <pyuipc/pyuipc.h>
#include <uipc/common/uipc.h>
#include <uipc/common/log.h>
#include <pyuipc/common/unit.h>
#include <pyuipc/common/uipc_type.h>
#include <pyuipc/common/timer.h>
#include <pyuipc/common/transform.h>
#include <pyuipc/common/logger.h>

#include <pyuipc/geometry/module.h>
#include <pyuipc/core/module.h>
#include <pyuipc/constitution/module.h>
#include <pyuipc/backend/module.h>
#include <pyuipc/builtin/module.h>
#include <pyuipc/diff_sim/module.h>

#include <pyuipc/backend/buffer_view.h>
#include <pyuipc/backend/buffer.h>
#include <pyuipc/core/feature.h>
#include <pyuipc/common/resident_thread.h>
#if UIPC_WITH_USD_SUPPORT
#include <pyuipc/usd/module.h>
#endif

using namespace uipc;

namespace pyuipc
{
static py::module* g_top_module = nullptr;

py::module& top_module()
{
    PYUIPC_ASSERT(g_top_module != nullptr, "top module is not initialized");
    return *g_top_module;
}
}  // namespace pyuipc

PYBIND11_MODULE(pyuipc, m)
{
    pyuipc::g_top_module = &m;

    auto unit         = m.def_submodule("unit");
    auto geometry     = m.def_submodule("geometry");
    auto constitution = m.def_submodule("constitution");
    auto diff_sim     = m.def_submodule("diff_sim");
    auto core         = m.def_submodule("core");
    auto backend      = m.def_submodule("backend");
    auto builtin      = m.def_submodule("builtin");
    auto usd          = m.def_submodule("usd");

    // pyuipc
    m.doc() = "Libuipc Python Binding";

    // define version
    m.attr("__version__") =
        fmt::format("{}.{}.{}", UIPC_VERSION_MAJOR, UIPC_VERSION_MINOR, UIPC_VERSION_PATCH);

    // # Workaround for MSVC Release Config
    // # Manually Convert Python Dict to Json
    m.def("init", [](py::dict dict) { uipc::init(pyjson::to_json(dict)); },
         py::arg("dict"),
         R"(Initialize the libuipc library with the given configuration.
Args:
    dict: Configuration dictionary containing library settings.)");

    m.def("default_config", &uipc::default_config,
         R"(Get the default configuration for libuipc.
Returns:
    dict: Default configuration dictionary.)");

    m.def("config", &uipc::config,
         R"(Get the current configuration of libuipc.
Returns:
    dict: Current configuration dictionary.)");


    pyuipc::PyUIPCType{m};
    pyuipc::PyLogger{m};
    pyuipc::PyTransform{m};
    pyuipc::PyTimer{m};
    pyuipc::PyResidentThread{m};

    // early expose buffer view
    pyuipc::backend::PyBufferView{backend};
    pyuipc::backend::PyBuffer{backend};
    // early expose feature
    pyuipc::core::PyFeature{core};

    // pyuipc.unit
    pyuipc::PyUnit{unit};

    // pyuipc.geometry
    pyuipc::geometry::PyModule{geometry};

    // pyuipc.constitution
    pyuipc::constitution::PyModule{constitution};

    // pyuipc::diff_sim
    pyuipc::diff_sim::PyModule{diff_sim};

    // pyuipc.core
    pyuipc::core::PyModule{core};

    // expose core classes to top level
    m.attr("Engine")    = core.attr("Engine");
    m.attr("World")     = core.attr("World");
    m.attr("Scene")     = core.attr("Scene");
    m.attr("SceneIO")   = core.attr("SceneIO");
    m.attr("Animation") = core.attr("Animation");

    // pyuipc.backend
    pyuipc::backend::PyModule{backend};

    // pyuipc.builtin
    pyuipc::builtin::PyModule{builtin};

    // pyuipc.usd
#if UIPC_WITH_USD_SUPPORT
    pyuipc::usd::PyModule{usd};
#endif
}
