#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::usd
{
class PyModule
{
  public:
    PyModule(py::module& m);
};
}  // namespace pyuipc::usd
