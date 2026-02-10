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
#include <pyuipc/constitution/constitution.h>
#include <pyuipc/geometry/attribute_slot.h>
#include <pyuipc/geometry/attribute_collection.h>
#include <pyuipc/geometry/geometry.h>
#include <pyuipc/geometry/implicit_geometry.h>
#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/geometry/geometry_atlas.h>
#include <pyuipc/geometry/geometry_slot.h>
#include <pyuipc/geometry/implicit_geometry_slot.h>
#include <pyuipc/geometry/simplicial_complex_slot.h>
#include <pyuipc/diff_sim/parameter_collection.h>
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

    m.def(
        "init",
        &uipc::init,
        py::arg("dict"),
        R"(Initialize the libuipc library with the given configuration.
Args:
    dict: Configuration dictionary containing library settings.)");

    m.def("default_config",
          &uipc::default_config,
          R"(Get the default configuration for libuipc.
Returns:
    dict: Default configuration dictionary.)");

    m.def("config",
          &uipc::config,
          R"(Get the current configuration of libuipc.
Returns:
    dict: Current configuration dictionary.)");


    pyuipc::PyUIPCType{m};
    pyuipc::PyLogger{m};
    pyuipc::PyTransform{m};
    pyuipc::PyTimer{m};
    pyuipc::PyResidentThread{m};

    // Early Expose Data Structures
    pyuipc::core::PyFeature{core};
    pyuipc::backend::PyBufferView{backend};
    pyuipc::backend::PyBuffer{backend};
    pyuipc::constitution::PyConstitution{constitution};
    pyuipc::geometry::PyAttributeSlot{geometry};
    pyuipc::geometry::PyAttributeCollection{geometry};
    pyuipc::geometry::PyGeometry{geometry};
    pyuipc::geometry::PyImplicitGeometry{geometry};
    pyuipc::geometry::PySimplicialComplex{geometry};
    pyuipc::geometry::PyGeometryAtlas{geometry};
    pyuipc::geometry::PyGeometrySlot{geometry};
    pyuipc::geometry::PyImplicitGeometrySlot{geometry};
    pyuipc::geometry::PySimplicialComplexSlot{geometry};
    // early expose diff_sim data structures
    pyuipc::diff_sim::PyParameterCollection{diff_sim};

    // pyuipc.unit
    pyuipc::PyUnit{unit};

    // pyuipc.core (must be before geometry utils as they depend on core types)
    pyuipc::core::PyModule{core};

    // pyuipc.geometry (utils/IO only, data structures exported early)
    pyuipc::geometry::PyModule{geometry};

    // pyuipc.constitution
    pyuipc::constitution::PyModule{constitution};

    // pyuipc::diff_sim
    pyuipc::diff_sim::PyModule{diff_sim};

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
