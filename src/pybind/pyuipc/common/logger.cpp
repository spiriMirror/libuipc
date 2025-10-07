#include <pyuipc/common/logger.h>
#include <uipc/common/logger.h>

namespace pyuipc
{
PyLogger::PyLogger(py::module& m)
{
    auto class_Logger = py::class_<uipc::Logger>(m, "Logger");
    auto enum_LoggerLevel = py::enum_<spdlog::level::level_enum>(class_Logger, "Level");

    enum_LoggerLevel.value("Trace", spdlog::level::trace)
        .value("Debug", spdlog::level::debug)
        .value("Info", spdlog::level::info)
        .value("Warn", spdlog::level::warn)
        .value("Error", spdlog::level::err)
        .value("Critical", spdlog::level::critical)
        .export_values();

    class_Logger.def_static("set_level",
                            [](spdlog::level::level_enum level)
                            { uipc::logger::set_level(level); });
}
}  // namespace pyuipc
