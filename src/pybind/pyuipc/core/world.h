#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyWorld
{
  public:
    PyWorld(py::module& m);
};
}  // namespace pyuipc::core
