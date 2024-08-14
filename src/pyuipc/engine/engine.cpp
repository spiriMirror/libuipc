#include <pyuipc/engine/engine.h>
#include <uipc/engine/engine.h>
#include <pyuipc/common/json.h>

namespace pyuipc::engine
{
using namespace uipc::engine;

class IPyEngine : public IEngine
{
  public:
    // Inherited via IEngine
    IPyEngine() = default;
    void do_init(backend::WorldVisitor v) override
    {
        PYBIND11_OVERRIDE_PURE(void, IEngine, do_init, v);
    }
    void do_advance() override
    {
        PYBIND11_OVERRIDE_PURE(void, IEngine, do_advance);
    }
    void do_sync() override { PYBIND11_OVERRIDE_PURE(void, IEngine, do_sync); }
    void do_retrieve() override
    {
        PYBIND11_OVERRIDE_PURE(void, IEngine, do_retrieve);
    }
    SizeT get_frame() const override
    {
        PYBIND11_OVERRIDE_PURE(SizeT, IEngine, get_frame);
    }
};

PyEngine::PyEngine(py::module& m)
{
    auto class_IEngine = py::class_<IEngine, IPyEngine>(m, "IEngine");
    class_IEngine.def(py::init<>());
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
        py::init(
            [](std::string_view backend_name, std::string_view workspace, const Json& config)
            { return std::make_unique<Engine>(backend_name, workspace, config); }),
        py::arg("backend_name"),
        py::arg("workspace") = "./",
        py::arg("config")    = Engine::default_config());
}
}  // namespace pyuipc::engine
