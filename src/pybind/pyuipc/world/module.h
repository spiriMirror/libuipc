#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::world
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::world
