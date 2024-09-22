#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyUIDRegister
{
  public:
    PyUIDRegister(py::module& m);
};
}  // namespace pyuipc::builtin
