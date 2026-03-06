#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
class PyModule
{
  public:
    PyModule(py::module_& m);
};
}  // namespace pyuipc::constitution
