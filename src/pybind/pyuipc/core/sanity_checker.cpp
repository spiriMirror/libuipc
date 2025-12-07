#include <pyuipc/core/sanity_checker.h>
#include <uipc/core/sanity_checker.h>
#include <pybind11/stl.h>

namespace pyuipc::core
{
using namespace uipc::core;
PySanityChecker::PySanityChecker(py::module& m)
{
    // define enum SanityCheckResult
    py::enum_<SanityCheckResult>(m, "SanityCheckResult", R"(Sanity check result enumeration.)")
        .value("Success", SanityCheckResult::Success)
        .value("Warning", SanityCheckResult::Warning)
        .value("Error", SanityCheckResult::Error)
        .export_values();

    auto class_SanityCheckMessage = py::class_<SanityCheckMessage>(
        m, "SanityCheckMessage", R"(SanityCheckMessage class representing a sanity check result message.)");
    class_SanityCheckMessage
        .def("id",
             &SanityCheckMessage::id,
             R"(Get the message ID.
Returns:
    int: Message ID.)")
        .def("name",
             &SanityCheckMessage::name,
             R"(Get the message name.
Returns:
    str: Message name.)")
        .def("result",
             &SanityCheckMessage::result,
             R"(Get the check result.
Returns:
    SanityCheckResult: Result of the check.)")
        .def("message",
             &SanityCheckMessage::message,
             R"(Get the message text.
Returns:
    str: Message text.)")
        .def("geometries",
             &SanityCheckMessage::geometries,
             R"(Get the list of geometry IDs associated with this message.
Returns:
    list: List of geometry IDs.)")
        .def("is_empty",
             &SanityCheckMessage::is_empty,
             R"(Check if the message is empty.
Returns:
    bool: True if empty, False otherwise.)");

    auto class_SanityChecker = py::class_<SanityChecker>(
        m, "SanityChecker", R"(SanityChecker class for validating scene and geometry data.)");

    class_SanityChecker.def("check", &SanityChecker::check)
        .def("report",
             &SanityChecker::report,
             R"(Get a report of all sanity check messages.
Returns:
    list: List of SanityCheckMessage objects.)")
        .def("errors",
             &SanityChecker::errors,
             R"(Get all error messages.
Returns:
    list: List of error SanityCheckMessage objects.)")
        .def("warns",
             &SanityChecker::warns,
             R"(Get all warning messages.
Returns:
    list: List of warning SanityCheckMessage objects.)")
        .def("infos",
             &SanityChecker::infos,
             R"(Get all info messages.
Returns:
    list: List of info SanityCheckMessage objects.)")
        .def("clear", &SanityChecker::clear, R"(Clear all sanity check messages.)");
}
}  // namespace pyuipc::core
