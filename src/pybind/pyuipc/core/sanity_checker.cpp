#include <pyuipc/core/sanity_checker.h>
#include <uipc/core/sanity_checker.h>
#include <pybind11/stl.h>

namespace pyuipc::core
{
using namespace uipc::core;
PySanityChecker::PySanityChecker(py::module& m)
{
    // define enum SanityCheckResult
    py::enum_<SanityCheckResult>(m, "SanityCheckResult")
        .value("Success", SanityCheckResult::Success)
        .value("Warning", SanityCheckResult::Warning)
        .value("Error", SanityCheckResult::Error)
        .export_values();

    auto class_SanityCheckMessage = py::class_<SanityCheckMessage>(m, "SanityCheckMessage");
    class_SanityCheckMessage.def("id", &SanityCheckMessage::id)
        .def("name", &SanityCheckMessage::name)
        .def("result", &SanityCheckMessage::result)
        .def("message", &SanityCheckMessage::message)
        .def("geometries", &SanityCheckMessage::geometries)
        .def("is_empty", &SanityCheckMessage::is_empty);

    auto class_SanityChecker = py::class_<SanityChecker>(m, "SanityChecker");
    class_SanityChecker.def("check", &SanityChecker::check)
        .def("report", &SanityChecker::report)
        .def("errors", &SanityChecker::errors)
        .def("warns", &SanityChecker::warns)
        .def("infos", &SanityChecker::infos)
        .def("clear", &SanityChecker::clear);
}
}  // namespace pyuipc::core
