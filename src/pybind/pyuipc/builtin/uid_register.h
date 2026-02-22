#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyUIDRegister
{
  public:
    PyUIDRegister(py::module_& m);
};
}  // namespace pyuipc::builtin
