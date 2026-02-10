#include <pyuipc/builtin/uid_info.h>
#include <uipc/builtin/uid_info.h>

namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyUIDInfo::PyUIDInfo(py::module& m)
{
    py::class_<UIDInfo>(m, "UIDInfo", R"(UIDInfo class containing information about a UID (unique identifier).)")
        .def_readwrite("uid", &UIDInfo::uid, R"(UID value.)")
        .def_readwrite("name", &UIDInfo::name, R"(Name associated with the UID.)")
        .def_readwrite("type", &UIDInfo::type, R"(Type of the UID.)")
        .def_readwrite("author", &UIDInfo::author, R"(Author of the UID.)")
        .def_readwrite("email", &UIDInfo::email, R"(Email of the author.)")
        .def_readwrite("website", &UIDInfo::website, R"(Website URL.)")
        .def_readwrite("description", &UIDInfo::description, R"(Description of the UID.)")
        .def_readwrite("extras", &UIDInfo::extras, R"(Extra JSON data.)")
        .def("is_official_builtin_uid", &UIDInfo::is_official_builtin_uid, py::arg("uid"), R"(Check if UID is an official builtin UID.)")
        .def("is_user_defined_uid", &UIDInfo::is_user_defined_uid, py::arg("uid"), R"(Check if UID is a user-defined UID.)")
        .def("to_json", &UIDInfo::to_json, R"(Convert to JSON representation.)");
}
}  // namespace pyuipc::builtin

