#include <pyuipc/core/engine.h>
#include <uipc/core/engine.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyEngine::PyEngine(py::module& m)
{
    auto class_Engine = py::class_<Engine>(m, "Engine");

    class_Engine
        .def(py::init(
                 [](std::string_view backend_name, std::string_view workspace, const Json& config) {
                     return std::make_unique<Engine>(backend_name, workspace, config);
                 }),
             py::arg("backend_name"),
             py::arg("workspace") = "./",
             py::arg("config")    = Engine::default_config())
        .def("backend_name", &Engine::backend_name);
}
}  // namespace pyuipc::core
