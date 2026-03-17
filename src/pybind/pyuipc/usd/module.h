#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::usd
{
class PyModule
{
  public:
    PyModule(py::module_& m);
};
}  // namespace pyuipc::usd
