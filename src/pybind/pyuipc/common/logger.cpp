#include <pyuipc/common/logger.h>
#include <uipc/common/logger.h>

namespace pyuipc
{
PyLogger::PyLogger(py::module& m)
{
    auto class_Logger = py::class_<uipc::Logger>(m, "Logger",
                                                   R"(Logger class for logging messages at different severity levels.)");
    auto enum_LoggerLevel = py::enum_<spdlog::level::level_enum>(class_Logger, "Level",
                                                                  R"(Logging level enumeration.)");

    enum_LoggerLevel.value("Trace", spdlog::level::trace)
        .value("Debug", spdlog::level::debug)
        .value("Info", spdlog::level::info)
        .value("Warn", spdlog::level::warn)
        .value("Error", spdlog::level::err)
        .value("Critical", spdlog::level::critical)
        .export_values();

    class_Logger.def_static("set_level",
                            [](spdlog::level::level_enum level)
                            { uipc::logger::set_level(level); },
                            py::arg("level"),
                            R"(Set the logging level.
Args:
    level: Logging level (Trace, Debug, Info, Warn, Error, Critical).)");

    class_Logger.def_static("set_pattern",
                            [](std::string_view pattern)
                            { uipc::logger::set_pattern(pattern); },
                            py::arg("pattern"),
                            R"(Set the logging pattern format.
Args:
    pattern: Format pattern string for log messages.)");

    class_Logger.def_static(
        "debug", [](std::string_view msg) { uipc::logger::debug(msg); },
        py::arg("msg"),
        R"(Log a debug message.
Args:
    msg: Message to log.)");
    class_Logger.def_static("info",
                            [](std::string_view msg) { uipc::logger::info(msg); },
                            py::arg("msg"),
                            R"(Log an info message.
Args:
    msg: Message to log.)");
    class_Logger.def_static("warn",
                            [](std::string_view msg) { uipc::logger::warn(msg); },
                            py::arg("msg"),
                            R"(Log a warning message.
Args:
    msg: Message to log.)");
    class_Logger.def_static(
        "error", [](std::string_view msg) { uipc::logger::error(msg); },
        py::arg("msg"),
        R"(Log an error message.
Args:
    msg: Message to log.)");
    class_Logger.def_static(
        "critical", [](std::string_view msg) { uipc::logger::critical(msg); },
        py::arg("msg"),
        R"(Log a critical message.
Args:
    msg: Message to log.)");
    class_Logger.def_static("log",
                            [](spdlog::level::level_enum level, std::string_view msg)
                            { uipc::logger::log(level, msg); },
                            py::arg("level"),
                            py::arg("msg"),
                            R"(Log a message at the specified level.
Args:
    level: Logging level.
    msg: Message to log.)");
}
}  // namespace pyuipc
