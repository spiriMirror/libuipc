#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
class PyResidentThread
{
  public:
    PyResidentThread(py::module& m);
};
}  // namespace pyuipc
