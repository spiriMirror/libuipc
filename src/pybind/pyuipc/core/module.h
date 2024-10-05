#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::core
