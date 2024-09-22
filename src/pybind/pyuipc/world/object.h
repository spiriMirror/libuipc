#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::world
{
class PyObject
{
  public:
    PyObject(py::module& m);
};
}  // namespace pyuipc::world
