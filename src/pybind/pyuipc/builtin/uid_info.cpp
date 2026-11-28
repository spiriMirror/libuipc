#include <pyuipc/builtin/uid_info.h>
#include <uipc/builtin/uid_info.h>

namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyUIDInfo::PyUIDInfo(py::module& m)
{
    py::class_<UIDInfo>(m, "UIDInfo", R"(Read-only metadata for a registered UID (unique identifier).)")
        .def_readonly("uid", &UIDInfo::uid, R"(UID value.)")
        .def_readonly("name", &UIDInfo::name, R"(Name associated with the UID.)")
        .def_readonly("type", &UIDInfo::type, R"(Type of the UID.)")
        .def_readonly("author", &UIDInfo::author, R"(Author of the UID.)")
        .def_readonly("email", &UIDInfo::email, R"(Email of the author.)")
        .def_readonly("website", &UIDInfo::website, R"(Website URL.)")
        .def_readonly("description", &UIDInfo::description, R"(Description of the UID.)")
        .def_readonly("extras", &UIDInfo::extras, R"(Extra JSON data.)")
        .def("is_official_builtin_uid",
             &UIDInfo::is_official_builtin_uid,
             py::arg("uid"),
             R"(Check if UID is an official builtin UID.)")
        .def("is_user_defined_uid", &UIDInfo::is_user_defined_uid, py::arg("uid"), R"(Check if UID is a user-defined UID.)")
        .def("to_json", &UIDInfo::to_json, R"(Convert to JSON representation.)");
}
}  // namespace pyuipc::builtin
