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
    auto class_EngineStatus = py::class_<EngineStatus, S<EngineStatus>>(m, "EngineStatus",
                                                                         R"(Engine status message indicating info, warning, or error.)");

    class_EngineStatus.def("type", &EngineStatus::type,
                           R"(Get the status type.
Returns:
    Type: Status type (None, Info, Warning, or Error).)")
        .def("what", &EngineStatus::what,
             R"(Get the status message.
Returns:
    str: Status message string.)")
        .def_static("info", &EngineStatus::info, py::arg("msg"),
                    R"(Create an info status.
Args:
    msg: Info message.
Returns:
    EngineStatus: Info status object.)")
        .def_static("warning", &EngineStatus::warning, py::arg("msg"),
                    R"(Create a warning status.
Args:
    msg: Warning message.
Returns:
    EngineStatus: Warning status object.)")
        .def_static("error", &EngineStatus::error, py::arg("msg"),
                    R"(Create an error status.
Args:
    msg: Error message.
Returns:
    EngineStatus: Error status object.)");

    // define enum
    py::enum_<EngineStatus::Type>(class_EngineStatus, "Type",
                                   R"(Engine status type enumeration.)")
        .value("None", EngineStatus::Type::None)
        .value("Info", EngineStatus::Type::Info)
        .value("Warning", EngineStatus::Type::Warning)
        .value("Error", EngineStatus::Type::Error)
        .export_values();

    auto class_EngineStatusCollection =
        py::class_<EngineStatusCollection, S<EngineStatusCollection>>(m, "EngineStatusCollection",
                                                                        R"(Collection of engine status messages.)");

    class_EngineStatusCollection.def(py::init<>(),
                                    R"(Create an empty status collection.)");
    class_EngineStatusCollection.def("push_back",
                                     [](EngineStatusCollection& self, const EngineStatus& status)
                                     { return self.push_back(status); },
                                     py::arg("status"),
                                     R"(Add a status to the collection.
Args:
    status: EngineStatus to add.)");
    class_EngineStatusCollection.def("has_error", &EngineStatusCollection::has_error,
                                      R"(Check if the collection contains any errors.
Returns:
    bool: True if any error status exists, False otherwise.)");
    class_EngineStatusCollection.def("to_json", &EngineStatusCollection::to_json,
                                      R"(Convert status collection to JSON.
Returns:
    dict: JSON representation of the status collection.)");

    auto class_IEngine = py::class_<IEngine, S<IEngine>>(m, "IEngine",
                                                          R"(Interface for engine implementations.)");

    auto class_PyIEngine =
        py::class_<PyIEngine, PyIEngine_, IEngine, S<PyIEngine>>(m, "PyIEngine",
                                                                  R"(Python-implementable engine interface.)");

    class_PyIEngine.def("do_init", [](PyIEngine& self) { self.do_init(); },
                        R"(Initialize the engine.)");

    class_PyIEngine.def(py::init<>(),
                        R"(Create a new PyIEngine instance.)");

    class_PyIEngine.def("do_advance", &PyIEngine::do_advance,
                        R"(Advance the simulation by one step.)")
        .def("do_sync", &PyIEngine::do_sync,
             R"(Synchronize engine state.)")
        .def("do_retrieve", &PyIEngine::do_retrieve,
             R"(Retrieve engine state.)")
        .def("do_to_json", &PyIEngine::do_to_json,
             R"(Convert engine state to JSON.
Returns:
    dict: JSON representation of engine state.)")
        .def("do_dump", &PyIEngine::do_dump,
             R"(Dump engine state.)")
        .def("do_recover", &PyIEngine::do_recover, py::arg("dst_frame"),
             R"(Recover engine state to a specific frame.
Args:
    dst_frame: Target frame number.)")
        .def("get_frame", &PyIEngine::get_frame,
             R"(Get the current frame number.
Returns:
    int: Current frame number.)")
        .def("status", &PyIEngine::get_status, py::return_value_policy::reference_internal,
             R"(Get engine status collection.
Returns:
    EngineStatusCollection: Reference to status collection.)")
        .def("features", &PyIEngine::get_features, py::return_value_policy::reference_internal,
             R"(Get engine features.
Returns:
    FeatureCollection: Reference to feature collection.)")
        .def("world", &PyIEngine::world, py::return_value_policy::reference_internal,
             R"(Get the world.
Returns:
    World: Reference to the world.)");


    auto class_Engine = py::class_<Engine, S<Engine>>(m, "Engine",
                                                     R"(Engine class for running simulations with a specific backend.)");
    class_Engine.def(py::init<std::string_view, std::string_view, const Json&>(),
                     py::call_guard<py::gil_scoped_release>(),
                     py::arg("backend_name"),
                     py::arg("workspace") = "./",
                     py::arg("config")    = Engine::default_config(),
                     R"(Create an engine with a backend.
Args:
    backend_name: Name of the backend to use (e.g., 'cuda', 'none').
    workspace: Workspace directory path (default: './').
    config: Configuration dictionary (optional, uses default if not provided).)");
    class_Engine.def(py::init<std::string_view, S<IEngine>, std::string_view, const Json&>(),
                     py::call_guard<py::gil_scoped_release>(),
                     py::arg("backend_name"),
                     py::arg("overrider"),
                     py::arg("workspace") = "./",
                     py::arg("config")    = Engine::default_config(),
                     R"(Create an engine with a custom engine overrider.
Args:
    backend_name: Name of the backend to use.
    overrider: Custom IEngine implementation.
    workspace: Workspace directory path (default: './').
    config: Configuration dictionary (optional, uses default if not provided).)");
    class_Engine.def("backend_name",
                     &Engine::backend_name,
                     py::call_guard<py::gil_scoped_release>(),
                     R"(Get the backend name.
Returns:
    str: Backend name.)");
    class_Engine.def("workspace", &Engine::workspace, py::call_guard<py::gil_scoped_release>(),
                     R"(Get the workspace directory.
Returns:
    str: Workspace directory path.)");
    class_Engine.def("features",
                     &Engine::features,
                     py::return_value_policy::reference_internal,
                     py::call_guard<py::gil_scoped_release>(),
                     R"(Get the feature collection.
Returns:
    FeatureCollection: Reference to feature collection.)");
    class_Engine.def_static("default_config",
                            &Engine::default_config,
                            py::call_guard<py::gil_scoped_release>(),
                            R"(Get the default engine configuration.
Returns:
    dict: Default configuration dictionary.)");
}
}  // namespace pyuipc::core
