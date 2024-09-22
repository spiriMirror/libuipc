#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class Module
{
  public:
    Module(py::module& m);
};
}  // namespace pyuipc::backend