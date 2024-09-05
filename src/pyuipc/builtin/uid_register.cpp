#include <pyuipc/builtin/uid_register.h>
#include <uipc/builtin/uid_register.h>
#include <pyuipc/common/json.h>

namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyUIDRegister::PyUIDRegister(py::module& m)
{
    py::class_<details::UIDRegister>(m, "UIDRegister")
        .def("find", &details::UIDRegister::find)
        .def("exists", &details::UIDRegister::exists)
        .def("to_json", &details::UIDRegister::to_json);
}
}  // namespace pyuipc::builtin
