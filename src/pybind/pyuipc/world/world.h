#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::world
{
class PyWorld
{
  public:
    PyWorld(py::module& m);
};
}  // namespace pyuipc::world
