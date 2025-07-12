#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
class PyFuture
{
  public:
    PyFuture(py::module& m);
};
}  // namespace pyuipc
