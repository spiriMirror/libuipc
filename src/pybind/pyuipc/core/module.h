#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyModule
{
  public:
    PyModule(py::module& m);
};
}  // namespace pyuipc::core
