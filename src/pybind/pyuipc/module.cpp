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


#include <pybind11/pybind11.h>
#include <mutex>

class MyClass
{
  public:
    MyClass() {}

    void lock_mutex()
    {
        py::gil_scoped_acquire      r;
        std::mutex                  my_mutex;  // Properly initialized mutex
        std::lock_guard<std::mutex> lock(my_mutex);
        // Critical section code here, accessing shared data
    }

    // If passing from Python:
    void set_mutex(std::mutex* mtx_ptr)
    {
        py::gil_scoped_release r;
        if(mtx_ptr)
        {
            my_mutex_ptr = mtx_ptr;
            // Lock through pointer.
            std::lock_guard<std::mutex> lock(*mtx_ptr);
        }
    }

  private:
    std::mutex* my_mutex_ptr = nullptr;  // Pointer
};

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

    // pyuipc
    m.doc() = "Libuipc Python Binding";

    // define version
    m.attr("__version__") =
        fmt::format("{}.{}.{}", UIPC_VERSION_MAJOR, UIPC_VERSION_MINOR, UIPC_VERSION_PATCH);

    // # Workaround for MSVC Release Config
    // # Manually Convert Python Dict to Json
    m.def("init", [](py::dict dict) { uipc::init(pyjson::to_json(dict)); });

    m.def("default_config", &uipc::default_config);

    m.def("config", &uipc::config);


    pyuipc::PyUIPCType{m};
    pyuipc::PyLogger{m};
    pyuipc::PyTransform{m};
    pyuipc::PyTimer{m};

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

    pybind11::class_<MyClass>(m, "MyClass")
        .def(pybind11::init<>())
        .def("lock_mutex", &MyClass::lock_mutex)
        .def("set_mutex", &MyClass::set_mutex);
}