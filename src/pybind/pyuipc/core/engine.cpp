#include <pyuipc/core/engine.h>
#include <uipc/core/engine.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyEngine::PyEngine(py::module& m)
{
    auto class_Engine = py::class_<Engine, S<Engine>>(m, "Engine");

    class_Engine
        .def(py::init<std::string_view, std::string_view, const Json&>(),
             py::call_guard<py::gil_scoped_acquire>(),
             py::arg("backend_name"),
             py::arg("workspace") = "./",
             py::arg("config")    = Engine::default_config())
        .def("backend_name", &Engine::backend_name)
        .def("workspace", &Engine::workspace)
        .def("features", &Engine::features, py::return_value_policy::reference_internal)
        .def_static("default_config", &Engine::default_config);
}
}  // namespace pyuipc::core
