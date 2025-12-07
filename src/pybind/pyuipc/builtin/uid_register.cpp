#include <pyuipc/builtin/uid_register.h>
#include <uipc/builtin/uid_register.h>
#include <pyuipc/common/json.h>

namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyUIDRegister::PyUIDRegister(py::module& m)
{
    py::class_<details::UIDRegister>(m, "UIDRegister",
                                      R"(UIDRegister class for registering and looking up UIDs (unique identifiers).)")
        .def("find", &details::UIDRegister::find,
             py::arg("uid"),
             R"(Find a UID in the register.
Args:
    uid: UID to find.
Returns:
    str or None: Name associated with UID if found, None otherwise.)")
        .def("exists", &details::UIDRegister::exists,
             py::arg("uid"),
             R"(Check if a UID exists in the register.
Args:
    uid: UID to check.
Returns:
    bool: True if UID exists, False otherwise.)")
        .def("to_json", &details::UIDRegister::to_json,
             R"(Convert register to JSON representation.
Returns:
    dict: JSON representation of the register.)");
}
}  // namespace pyuipc::builtin
