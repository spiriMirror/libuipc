#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::constitution
