#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
class PyLogger
{
  public:
    PyLogger(py::module_& m);
};
}  // namespace pyuipc
