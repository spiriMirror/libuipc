#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyObject
{
  public:
    PyObject(py::module& m);
};
}  // namespace pyuipc::core
