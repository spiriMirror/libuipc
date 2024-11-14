#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
class PyModule
{
  public:
    PyModule(py::module& m);
};
}  // namespace pyuipc::constitution
