#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::engine
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::world
