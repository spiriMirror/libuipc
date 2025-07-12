#include <pyuipc/core/engine.h>
#include <uipc/core/engine.h>
#include <pyuipc/common/json.h>
#include <uipc/core/world.h>
#include <pyuipc/core/pyengine.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyEngine::PyEngine(py::module& m)
{
    auto class_EngineStatus = py::class_<EngineStatus, S<EngineStatus>>(m, "EngineStatus");

    class_EngineStatus.def("type", &EngineStatus::type)
        .def("what", &EngineStatus::what)
        .def_static("info", &EngineStatus::info, py::arg("msg"))
        .def_static("warning", &EngineStatus::warning, py::arg("msg"))
        .def_static("error", &EngineStatus::error, py::arg("msg"));

    // define enum
    py::enum_<EngineStatus::Type>(class_EngineStatus, "Type")
        .value("None", EngineStatus::Type::None)
        .value("Info", EngineStatus::Type::Info)
        .value("Warning", EngineStatus::Type::Warning)
        .value("Error", EngineStatus::Type::Error)
        .export_values();

    auto class_EngineStatusCollection =
        py::class_<EngineStatusCollection, S<EngineStatusCollection>>(m, "EngineStatusCollection");

    class_EngineStatusCollection.def(py::init<>());
    class_EngineStatusCollection.def("push_back",
                                     [](EngineStatusCollection& self, const EngineStatus& status)
                                     { return self.push_back(status); });
    class_EngineStatusCollection.def("has_error", &EngineStatusCollection::has_error);
    class_EngineStatusCollection.def("to_json", &EngineStatusCollection::to_json);

    auto class_IEngine = py::class_<IEngine, S<IEngine>>(m, "IEngine");

    auto class_PyIEngine =
        py::class_<PyIEngine, PyIEngine_, IEngine, S<PyIEngine>>(m, "PyIEngine");

    class_PyIEngine.def("do_init", [](PyIEngine& self) { self.do_init(); });

    class_PyIEngine.def(py::init<>());

    class_PyIEngine.def("do_advance", &PyIEngine::do_advance)
        .def("do_sync", &PyIEngine::do_sync)
        .def("do_retrieve", &PyIEngine::do_retrieve)
        .def("do_to_json", &PyIEngine::do_to_json)
        .def("do_dump", &PyIEngine::do_dump)
        .def("do_recover", &PyIEngine::do_recover, py::arg("dst_frame"))
        .def("get_frame", &PyIEngine::get_frame)
        .def("status", &PyIEngine::get_status, py::return_value_policy::reference_internal)
        .def("features", &PyIEngine::get_features, py::return_value_policy::reference_internal)
        .def("world", &PyIEngine::world, py::return_value_policy::reference_internal);


    auto class_Engine = py::class_<Engine, S<Engine>>(m, "Engine");
    class_Engine.def(py::init<std::string_view, std::string_view, const Json&>(),
                     py::call_guard<py::gil_scoped_release>(),
                     py::arg("backend_name"),
                     py::arg("workspace") = "./",
                     py::arg("config")    = Engine::default_config());
    class_Engine.def(py::init<std::string_view, S<IEngine>, std::string_view, const Json&>(),
                     py::call_guard<py::gil_scoped_release>(),
                     py::arg("backend_name"),
                     py::arg("overrider"),
                     py::arg("workspace") = "./",
                     py::arg("config")    = Engine::default_config());
    class_Engine.def("backend_name",
                     &Engine::backend_name,
                     py::call_guard<py::gil_scoped_release>());
    class_Engine.def("workspace", &Engine::workspace, py::call_guard<py::gil_scoped_release>());
    class_Engine.def("features",
                     &Engine::features,
                     py::return_value_policy::reference_internal,
                     py::call_guard<py::gil_scoped_release>());
    class_Engine.def_static("default_config",
                            &Engine::default_config,
                            py::call_guard<py::gil_scoped_release>());
}
}  // namespace pyuipc::core
