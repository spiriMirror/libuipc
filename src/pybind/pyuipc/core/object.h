#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyObject
{
  public:
    PyObject(py::module_& m);
};
}  // namespace pyuipc::core
