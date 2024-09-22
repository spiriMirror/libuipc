#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
class PyLogger
{
  public:
    PyLogger(py::module& m);
};
}  // namespace pyuipc
