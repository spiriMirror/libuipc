#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyModule
{
  public:
    PyModule(py::module& m);
};
}  // namespace pyuipc::backend