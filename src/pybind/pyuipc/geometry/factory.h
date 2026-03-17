#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::geometry
{
class PyFactory
{
  public:
    PyFactory(py::module_& m);
};
}  // namespace pyuipc::geometry
