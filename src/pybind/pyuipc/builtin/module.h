#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::constitution
