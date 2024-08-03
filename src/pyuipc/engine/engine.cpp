#include <pyuipc/engine/engine.h>
#include <uipc/engine/engine.h>
#include <pyuipc/common/json.h>

namespace pyuipc::engine
{
using namespace uipc::engine;

PyEngine::PyEngine(py::module& m)
{
    auto class_IEngine = py::class_<IEngine>(m, "IEngine");
    class_IEngine.def("init", &IEngine::init);
    class_IEngine.def("advance", &IEngine::advance);
    class_IEngine.def("sync", &IEngine::sync);
    class_IEngine.def("retrieve", &IEngine::retrieve);
    class_IEngine.def("to_json", &IEngine::to_json);
    class_IEngine.def("dump", &IEngine::dump);
    class_IEngine.def("recover", &IEngine::recover);
    class_IEngine.def("frame", &IEngine::frame);

    auto class_Engine = py::class_<Engine, IEngine>(m, "Engine");

    class_Engine.def(
        py::init([](std::string_view backend_name, std::string_view workspace)
                 { return std::make_unique<Engine>(backend_name, workspace); }),
        py::arg("backend_name"),
        py::arg("workspace") = "./");
}
}  // namespace pyuipc::engine
